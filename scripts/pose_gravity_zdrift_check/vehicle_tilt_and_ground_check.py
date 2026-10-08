#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""★决定性判据（不需要 lidar→IMU 外参、不需要 SLAM/地图）★
回答两个问题：
  Q-A 车辆【物理姿态】相对重力到底变过多少？          （= 有没有真的上/下坡）
  Q-B 地面在【雷达系】里平不平、传感器离地多高、前方坡度多少？（不用外参）

为什么必须用它：`lidar_ground_vs_gravity.py` 算的是"地面法向 vs 加计重力"，而
"加计重力→雷达系"这一步要用 lidar→IMU 外参；外参若在倾斜上有 e° 误差，
该判据就会把 e° 全部算成"坡道"。本脚本用两个都【不需要外参】的物理量交叉验证：

  Q-A：加计（低动态窗口）给出重力在 IMU 系的方向；扣掉【静止零偏】的陀螺积分给出
       车身系相对起始的姿态变化。二者都只在 IMU 系内部比较 ⇒ 与外参无关。
       · 平路/缓坡行驶 ⇒ 姿态变化 < 几度
       · 真走 20° 坡    ⇒ 必然出现 ~20° 变化（返回时复原）
  Q-B：地面在雷达 +y 直方图上必然是【尖峰】（水平面 + 固定安装高度），
       由峰值定"地面高度"，再在该高度拟合平面 ⇒ 得到地面在雷达系的倾角与
       传感器离地高度；这个量只涉及同一刚体单元内部，与安装姿态无关。

用法：
    source /opt/ros/humble/setup.bash
    python3 scripts/pose_gravity_zdrift_check/vehicle_tilt_and_ground_check.py \
        --bag /home/h/ddh/bag/930pm/bag/jieya/rosbag2_2026_10_01-00_49_15

判读：
    Q-A 全程姿态变化 < 几度 ⇒ 车体从未大幅倾斜 ⇒ 不存在 20° 级坡道；
    Q-B 离地高恒定（±2 cm）且地面在雷达系里接近水平 ⇒ 车一路贴地、地面本身水平。
"""
import argparse
import math
import sys

import numpy as np

TMAP = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
G = 9.80665


def ang(u, v):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u, v))))))


def q2R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def cloud_xyz(msg):
    """PointCloud2 → (N,3)（按 offset/point_step 正确解析，含 padding；NaN 由调用方排除）"""
    dt = np.dtype({'names': [f.name for f in msg.fields],
                   'formats': [TMAP[f.datatype] for f in msg.fields],
                   'offsets': [f.offset for f in msg.fields],
                   'itemsize': msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)


def read_imu(bag, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    T, A, W = [], [], []
    while r.has_next():
        _, d, t = r.read_next()
        m = deserialize_message(d, Imu)
        T.append(t * 1e-9)
        A.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        W.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    T = np.array(T)
    A = np.array(A)
    W = np.array(W)
    o = np.argsort(T)
    return T[o], A[o], W[o]


def read_frames(bag, topic, min_pts=1000):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while r.has_next():
        _, d, t = r.read_next()
        m = deserialize_message(d, PointCloud2)
        if m.width * m.height >= min_pts:
            out.append((t * 1e-9, m))
    return out


def part_a(T, A, W, static_s, win_s, quiet_gyr):
    """车辆物理姿态相对重力的变化（加计 + 扣零偏陀螺积分），全程只需要 IMU"""
    n0 = np.linalg.norm(A[T < T[0] + static_s].mean(0))
    sc = G / n0                              # 本平台加计读数是 g 单位；若已是 m/s² 也兼容
    bg = W[T < T[0] + static_s].mean(0)
    bg_end = W[T > T[-1] - 2.0].mean(0)
    print('[Q-A] 加计换算系数 %.4f（|a0_raw| = %.4f）' % (sc, n0))
    print('      陀螺静止零偏: 开头 %s | 结尾 %s（|.| %.5f / %.5f rad/s ⇒ 零偏稳定，可整体扣除）'
          % (np.round(bg, 5), np.round(bg_end, 5), np.linalg.norm(bg), np.linalg.norm(bg_end)))
    u0 = A[T < T[0] + static_s].mean(0)
    u0 = u0 / np.linalg.norm(u0)
    dt = np.diff(T)
    q = np.array([1.0, 0, 0, 0])
    Q = np.zeros((len(T), 4))
    Q[0] = q
    for i in range(1, len(T)):
        w = 0.5 * (W[i] + W[i - 1]) - bg
        n = np.linalg.norm(w)
        if n > 1e-12:
            dq = np.array([math.cos(n * dt[i - 1] / 2), *(math.sin(n * dt[i - 1] / 2) * w / n)])
            q = np.array([q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3],
                          q[0] * dq[1] + q[1] * dq[0] + q[2] * dq[3] - q[3] * dq[2],
                          q[0] * dq[2] - q[1] * dq[3] + q[2] * dq[0] + q[3] * dq[1],
                          q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1] + q[3] * dq[0]])
            q = q / np.linalg.norm(q)
        Q[i] = q
    print('        t(s)  |a|(m/s²) |w|扣零偏  姿态变化(加计) 姿态变化(陀螺)  两者差')
    rows = []
    for i in range(int((T[-1] - T[0]) / win_s)):
        m = (T - T[0] >= i * win_s) & (T - T[0] < (i + 1) * win_s)
        if m.sum() < 8:
            continue
        aw = A[m].mean(0)
        na = np.linalg.norm(aw) * sc
        ww = W[m].mean(0) - bg
        u = aw / np.linalg.norm(aw)
        up = q2R(Q[int(np.argmax(m))]).T @ u0
        if abs(na - G) < 0.35 and np.linalg.norm(ww) < quiet_gyr:
            am, ag = ang(u, u0), ang(up, u0)
            rows.append((T[m][0] - T[0], am, ag))
            print('      %5.2f   %7.3f   %.4f      %6.2f        %6.2f       %5.2f'
                  % (T[m][0] - T[0], na, np.linalg.norm(ww), am, ag, ang(u, up)))
    if rows:
        a = np.array(rows)
        k = int(np.argmax(a[:, 1]))
        print('      ⇒ 低动态窗口 %d 个：加计测得姿态变化 中位 %.2f°/最大 %.2f°(t=%.2f s)；'
              '陀螺积分 中位 %.2f°/最大 %.2f°'
              % (len(a), float(np.median(a[:, 1])), float(a[k, 1]), float(a[k, 0]),
                 float(np.median(a[:, 2])), float(a[:, 2].max())))
        print('      ⇒ 判读：真走 20° 坡这里必然出现 ~20° 变化；只有几度 ⇒ 车体从未大幅倾斜。')
    a0 = A[T < T[0] + static_s].mean(0)
    return a0 / np.linalg.norm(a0)          # 静止窗口的重力反方向(IMU 系)，供 Q-B 换算用


def part_b(frames, rmax, band, up_L=None):
    """地面在雷达系的倾角 / 传感器离地高度 / 前方坡度（用 +y 直方图峰定地面位置）"""
    print('\n[Q-B] 地面（雷达系）：用 +y 直方图尖峰定地面高度，再在该高度拟合平面')
    print('      帧   t(s)  离地高(m)  地面法向(雷达系)        与+y夹角 沿z坡度 沿x坡度 rms(m) 点数')
    rows = []
    for k in range(0, len(frames), max(1, len(frames) // 12)):
        t, m = frames[k]
        P = cloud_xyz(m)
        P = P[~np.isnan(P).any(1)]                       # ★ 必须排除 NaN 无效点
        P = P[np.linalg.norm(P, axis=1) < rmax]
        if len(P) < 500:
            continue
        y = P[:, 1]
        cnt, edg = np.histogram(y, bins=np.arange(-3.0, -0.05, 0.05))
        j = int(np.argmax(cnt))
        yf = 0.5 * (edg[j] + edg[j + 1])                 # 地面高度（尖峰=水平地面）
        S = P[np.abs(y - yf) < band]
        if len(S) < 200:
            continue
        cen = S.mean(0)
        n = np.linalg.svd(S - cen)[2][2]
        if n[1] < 0:
            n = -n                                        # 法向朝上(+y)
        rms = float(np.sqrt(np.mean(((S - cen) @ n) ** 2)))
        sz = math.degrees(math.atan(-n[2] / n[1]))        # 沿 +z(前) 的坡度
        sx = math.degrees(math.atan(-n[0] / n[1]))        # 沿 +x(左) 的坡度
        rows.append((t - frames[0][0], -yf, ang(n, [0, 1, 0]), sz, sx))
        extra = ''
        if up_L is not None:
            extra = '  (与 up_L 夹角 %.2f° = 已含外参倾斜误差)' % ang(n, up_L)
        print('      %3d %6.2f   %+.3f   [%6.3f %6.3f %6.3f]  %5.2f° %+6.2f° %+6.2f° %.4f %5d%s'
              % (k, t - frames[0][0], -yf, n[0], n[1], n[2], ang(n, [0, 1, 0]), sz, sx, rms, len(S), extra))
    if rows:
        a = np.array(rows)
        print('      ⇒ 离地高 中位 %.3f m（范围 %.3f~%.3f，跨度 %.3f m）⇒ 车是否始终贴地'
              % (np.median(a[:, 1]), a[:, 1].min(), a[:, 1].max(), a[:, 1].max() - a[:, 1].min()))
        print('      ⇒ 地面法向与雷达 +y 夹角 中位 %.2f°（≤几度 ⇒ 设计假设"雷达 y=上"成立、地面在雷达系里接近水平）'
              % float(np.median(a[:, 2])))
        print('      ⇒ 前方坡度 dy/dz 中位 %+.2f°（范围 %+.2f~%+.2f°）⇒ 地形相对车体的坡度'
              % (np.median(a[:, 3]), a[:, 3].min(), a[:, 3].max()))
        print('      ⇒ 与 LIO 日志里 z 的下降速率（如 0.42 m/m = 22.8°）对比：地形只有几度 ⇒ 那部分 z 变化是假的。')
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--imu-topic', default='/lidar/front/imu')
    ap.add_argument('--points-topic', default='/lidar/front/points_raw')
    ap.add_argument('--static-window', type=float, default=0.6)
    ap.add_argument('--win', type=float, default=0.1)
    ap.add_argument('--quiet-gyr', type=float, default=0.08, help='低动态窗口的 |ω|（扣零偏后）门限')
    ap.add_argument('--rmax', type=float, default=9.0)
    ap.add_argument('--band', type=float, default=0.15, help='地面高度带半宽(m)')
    ap.add_argument('--extrinsic-r', default='0,-1,0,1,0,0,0,0,1',
                    help='仅用于附带打印"与 up_L 夹角"（默认=配置里的 lidar→IMU）')
    args = ap.parse_args()

    T, A, W = read_imu(args.bag, args.imu_topic)
    if len(T) == 0:
        print('没有 IMU：检查 --imu-topic')
        return 1
    print('IMU %d 帧, %.2f s, %.1f Hz' % (len(T), T[-1] - T[0], len(T) / (T[-1] - T[0])))
    u0 = part_a(T, A, W, args.static_window, args.win, args.quiet_gyr)

    R = np.array([float(x) for x in args.extrinsic_r.split(',')], float).reshape(3, 3)
    up_L = R.T @ u0
    print('\n（附）配置外参下的重力反方向 up_L(雷达系) = %s；它与雷达 +y 的夹角 = %.2f°'
          % (np.round(up_L, 4), ang(up_L, [0, 1, 0])))
    print('      ⇒ 若外参在倾斜上有 e° 误差，"地面 vs 重力"就会被算成 e° 级的"坡"（旧判据的坑）。')

    frames = read_frames(args.bag, args.points_topic)
    if not frames:
        print('没有点云：检查 --points-topic')
        return 1
    print('点云帧 %d 个' % len(frames))
    part_b(frames, args.rmax, args.band, up_L)
    return 0


if __name__ == '__main__':
    sys.exit(main())
