#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""【辅助判据】"地面法向 vs 加计重力"的夹角 —— ★但它依赖 lidar→IMU 外参，不能单独定"有没有坡"

原理：IMU 内置在雷达里（同一刚体），因此可以在【同一个单元内部】比较两个量：
    · n_L ：雷达原始点云拟合出的地面法向（雷达系）
    · up_L：加计静止窗口的比力方向，经 extrinsic_R 转到雷达系 = 重力方向（雷达系）
  两者夹角 = 地面相对【重力】的倾角。

⚠⚠ 2026-10-02 重要更正（本 bag 实测教训，务必先读）⚠⚠
  这个"倾角"里【已经含进了 lidar→IMU 外参的倾斜误差】：up_L = R_cfg^T·â0，外参若在倾斜上
  有 e° 误差，本判据就会把 e° 全部读成"坡"。旧版还额外踩了一个坑：它按"离传感器最近的平面"
  当地面 —— 本 bag 里恰好有一层 11° 倾斜的面（0.44 m 远、rms 7 mm、延伸 10 m），于是读出
  22.74°（= 11.33° 外参/安装倾角 + 11.2° 选错面），并据此下了"车确实行驶在 22.7° 真实坡道"的
  错误结论。同时旧版没排 NaN（本 bag 20.6% 无回波点），RANSAC 还会拟合出"伪平面"。
  修正后（见本文件 ground_normal()）：先排除 NaN，再用"雷达 +y 直方图尖峰"定地面高度、
  只在该高度附近拟合平面 ⇒ 本 bag 实测：地面法向与雷达 +y 只差 0.24~0.37°，沿行驶方向坡度
  +0.09~+0.27°，离地高全程 0.525 m 恒定；与 up_L 的夹角 11.1~11.3°（= 外参/安装倾角量级）。

⇒ 结论：**本判据只能用来核对"外参/安装倾角 + 地面坡度"的合量，不能单独判定地形**。
   判定"车到底有没有上/下坡"请用 **vehicle_tilt_and_ground_check.py**（完全不需要外参）：
     · Q-A 车辆物理姿态相对重力的变化（加计 + 扣零偏陀螺积分）——真走 20° 坡必然出现 ~20° 变化；
     · Q-B 地面在雷达系的坡度与离地高度——本 bag 实测坡度 ≤0.3°、离地高恒定 0.525 m。

用法：
    source /opt/ros/humble/setup.bash
    python3 scripts/pose_gravity_zdrift_check/lidar_ground_vs_gravity.py \
        --bag /home/h/ddh/bag/930pm/bag/jieya/rosbag2_2026_10_01-00_49_15 \
        --extrinsic-r 0,-1,0,1,0,0,0,0,1
"""
import argparse
import math
import sys

import numpy as np


def cloud_xyz(msg):
    """PointCloud2 → (N,3) float64（只取 x/y/z）"""
    tmap = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    dt = np.dtype([(f.name, tmap[f.datatype]) for f in msg.fields])
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([a['x'].astype(np.float64), a['y'].astype(np.float64), a['z'].astype(np.float64)], 1)


def ransac_plane(P, thr=0.04, iters=600, seed=0):
    """返回 (质心, 单位法向, 内点数, rms, 内点掩码)。"""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i = rng.choice(len(P), 3, replace=False)
        n = np.cross(P[i[1]] - P[i[0]], P[i[2]] - P[i[0]])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        c = int((np.abs((P - P[i[0]]) @ n) < thr).sum())
        if best is None or c > best[0]:
            best = (c, n, P[i[0]])
    _, n, p0 = best
    inl = np.abs((P - p0) @ n) < thr
    Q = P[inl]
    cen = Q.mean(0)
    n = np.linalg.svd(Q - cen)[2][2]
    rms = float(np.sqrt(np.mean(((Q - cen) @ n) ** 2)))
    return cen, n, int(inl.sum()), rms, inl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True, help='rosbag2 目录')
    ap.add_argument('--imu-topic', default='/lidar/front/imu')
    ap.add_argument('--points-topic', default='/lidar/front/points_raw')
    ap.add_argument('--extrinsic-r', default='0,-1,0,1,0,0,0,0,1',
                    help='fasterlio.extrinsic_R 行主序 9 个数（lidar→IMU：p_imu = R p_lidar）')
    ap.add_argument('--static-window', type=float, default=0.6, help='取开头这么久的 IMU 作为静止窗口(s)')
    ap.add_argument('--rmax-near', type=float, default=3.0, help='近场半径(m)：车正下方的地面')
    ap.add_argument('--rmax-far', type=float, default=9.0, help='远场半径(m)：车前方的地面')
    ap.add_argument('--max-frames', type=int, default=400)
    ap.add_argument('--sample', type=int, default=6, help='打印多少帧（均匀取样）')
    args = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2

    R = np.array([float(x) for x in args.extrinsic_r.split(',')], float).reshape(3, 3)

    # --- 1) 静止窗口的比力方向 = 重力方向（IMU 系 → 雷达系） ---
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.imu_topic]))
    T, A = [], []
    while reader.has_next():
        _, data, t = reader.read_next()
        m = deserialize_message(data, Imu)
        T.append(t * 1e-9)
        A.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    if not T:
        print('没有 IMU 数据（检查 --imu-topic）')
        return 1
    T = np.array(T)
    A = np.array(A)
    sel = T < T[0] + args.static_window
    a0_raw = A[sel].mean(0)
    a0 = a0_raw / np.linalg.norm(a0_raw)
    up_L = R.T @ a0
    print('静止窗口加计均值(IMU系) â0 = %s   (|a| = %.4f m/s²，理想 9.80665)' % (np.round(a0, 4), np.linalg.norm(a0_raw)))
    print('⇒ 重力方向在【雷达系】up_L = %s' % np.round(up_L, 4))
    print('   与雷达 +y(设计上的“上”)夹角 %.2f deg；与雷达 +z 夹角 %.2f deg'
          % (math.degrees(math.acos(min(1.0, abs(up_L[1])))), math.degrees(math.acos(min(1.0, abs(up_L[2]))))))

    # --- 2) 原始点云帧：地面法向（远场/近场） ---
    reader2 = rosbag2_py.SequentialReader()
    reader2.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                 rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader2.set_filter(rosbag2_py.StorageFilter(topics=[args.points_topic]))
    frames = []
    while reader2.has_next() and len(frames) < args.max_frames:
        _, data, t = reader2.read_next()
        m = deserialize_message(data, PointCloud2)
        if m.width * m.height > 1000:
            frames.append((t * 1e-9, m))
    if not frames:
        print('没有点云数据（检查 --points-topic）')
        return 1
    print('\n点云帧 %d 个, 时间 %.2f~%.2f s' % (len(frames), frames[0][0] - T[0], frames[-1][0] - T[0]))

    def ground_normal(m, rmax):
        """★2026-10-02 修正版：稳健找地面，不再用"离传感器最近的平面"当地面。

        为什么改（本 bag 实测的教训）：
          · 旧写法在 sub-3 m 内先按 `P@up_L < -0.15`（沿 up_L 取"下方"）筛点，再取**离传感器最近的平面**当地面。
            up_L 一旦有 e° 倾斜（外参误差），"下方"这个半空间本身就是歪的；而"最近"会挑中
            **车体/结构上那层 11° 倾斜的面**（本 bag 真实存在：0.44 m 远、rms 7 mm、延伸 10 m），
            于是算出 22.7° 的假坡道（= 11.3° 外参倾斜 + 11.2° 选错面）。
          · 正确做法：地面在同一刚体单元里是【水平面 + 固定安装高度】⇒ 沿雷达 +y 的点云分布
            必然出现【尖峰】。先用尖峰定"地面高度"，只在该高度 ±band 内取点拟合平面。
            这个过程不需要任何外参，也不受其它斜面干扰。
          · 另外必须排除 NaN(本 bag 占 20.6%)，否则 RANSAC 会拟合出"伪平面"。
        """
        P = cloud_xyz(m)
        P = P[~np.isnan(P).any(1)]                     # ★ 排除 NaN 无效点
        P = P[np.linalg.norm(P, axis=1) < rmax]
        if len(P) < 500:
            return None
        y = P[:, 1]                                    # 雷达 +y = 设计上的"上"
        cnt, edg = np.histogram(y, bins=np.arange(-3.0, -0.05, 0.05))
        j = int(np.argmax(cnt))
        y_floor = 0.5 * (edg[j] + edg[j + 1])          # 地面高度（尖峰）
        S = P[np.abs(y - y_floor) < 0.15]
        if len(S) < 200:
            return None
        cen = S.mean(0)
        n = np.linalg.svd(S - cen)[2][2]
        if float(n @ up_L) < 0:
            n = -n
        rms = float(np.sqrt(np.mean(((S - cen) @ n) ** 2)))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(n @ up_L)))))
        cen.flat[1] = y_floor                          # 用尖峰高度当"地面高度"（离地高 = -y_floor）
        return cen, n, int(len(S)), rms, ang

    idx = np.unique(np.linspace(0, len(frames) - 1, args.sample).astype(int))
    for label, rmax in (('远场 <=%gm' % args.rmax_far, args.rmax_far),
                        ('近场 <=%gm（车正下方）' % args.rmax_near, args.rmax_near)):
        print('\n=== %s 的地面法向 vs 重力（= 地面真实倾角） ===' % label)
        print('  t(s)  地面法向(雷达系)             点数    rms(m)  与+y夹角  与 up_L 夹角(deg)  离地高(m)')
        angs, angs_y, hh = [], [], []
        for k in idx:
            t, m = frames[k]
            g = ground_normal(m, rmax)
            if g is None:
                continue
            cen, n, cnt, rms, ang = g
            a_y = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(n[1]))))))
            angs.append(ang)
            angs_y.append(a_y)
            hh.append(-float(cen[1]))
            print(' %5.2f  [%6.3f %6.3f %6.3f]  %6d  %.4f    %5.2f     %6.2f          %+.3f'
                  % (t - T[0], n[0], n[1], n[2], cnt, rms, a_y, ang, -float(cen[1])))
        if angs:
            print('  ⇒ 与 up_L 夹角 中位 %.2f°；与雷达 +y 夹角 中位 %.2f°（≤几度 ⇒ 设计假设"雷达 y=上"成立）；'
                  '离地高 中位 %.3f m（跨度 %.3f m）'
                  % (float(np.median(angs)), float(np.median(angs_y)), float(np.median(hh)),
                     max(hh) - min(hh)))

    print('''
判读（2026-10-02 修正）：
 · 本脚本的"地面 vs 重力"= 地面法向 与 up_L 的夹角，而 up_L = R_cfg^T·â0 依赖 lidar→IMU 外参。
   外参若在倾斜上有 e° 误差，这里就会把 e° 全部读成"坡" ⇒ **本判据不能单独用来判定"有没有坡"**。
 · 本 bag 实测（修正版判据）：地面法向与雷达 +y 只差 0.24~0.37°，沿行驶方向坡度 +0.09~+0.27°，
   离地高全程 0.525 m 恒定；而与 up_L 的夹角是 11.1~11.3° ⇒ 那个角度就是外参/安装倾角的量级。
   旧版还因为"取离传感器最近的平面当地面"选到一层 11° 斜面，才读出 22.7°（= 11.3+11.2）。
 · **要判定"车到底有没有上/下坡"，请用 vehicle_tilt_and_ground_check.py**（不需要外参）：
   那里同时给出"车辆物理姿态相对重力的变化"与"地面在雷达系的坡度"。
''')
    return 0


if __name__ == '__main__':
    sys.exit(main())

