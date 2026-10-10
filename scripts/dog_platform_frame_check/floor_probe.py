#!/usr/bin/env python3
"""只读探测（2026-10-09 狗平台排查第 3 步）：
1) 从给定 bag 取前若干帧点云，RANSAC 出"离传感器 0.15~0.9 m 的最大平面"= 地面；
   法向符号由【传感器在平面哪一侧】唯一确定（传感器在原点，点在平面上 ⇒ 上 = -n，且 d>0）。
   该判据不依赖加速度计，因此可独立判定"加计比力是不是朝上"。
2) 取静止窗口 IMU 均值 â，打印 angle(up_floor, â)：≈0° ⇒ 加计朝上(标准)；≈180° ⇒ 加计朝下(需翻符号)。
3) 由 tf_static 链 base_link→rslidartmp1→laser_points_front 算 up_urdf = R_base_laser^T ẑ 作二次独立参照。

配套报告：doc/dog_platform_frame_fix_report.md（本脚本原为 /tmp/floor_probe.py，原样归档；
本平台 ground_normal_lidar 的实测值就是它给出的，输出见报告第 2 节）
"""
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

BAG = "/home/h/ddh/bag/dog/20261009/短途/jieya/rosbag2_2026_10_09-10_50_41"
N_FRAMES = 6
IMU_N = 120


def rot(rx, ry, rz, rw):
    n = np.sqrt(rx * rx + ry * ry + rz * rz + rw * rw)
    rx, ry, rz, rw = rx / n, ry / n, rz / n, rw / n
    return np.array([
        [1 - 2 * (ry * ry + rz * rz), 2 * (rx * ry - rz * rw), 2 * (rx * rz + ry * rw)],
        [2 * (rx * ry + rz * rw), 1 - 2 * (rx * rx + rz * rz), 2 * (ry * rz - rx * rw)],
        [2 * (rx * rz - ry * rw), 2 * (ry * rz + rx * rw), 1 - 2 * (rx * rx + ry * ry)],
    ])


def ang(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    return np.degrees(np.arccos(np.clip(a.dot(b), -1.0, 1.0)))


def main():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
           rosbag2_conv("cdr", "cdr"))
    pts = []
    accs = []
    tfs = {}
    while r.has_next() and (len(pts) < N_FRAMES or len(accs) < IMU_N or not tfs):
        topic, data, _ = r.read_next()
        if topic == "/lidar/front/points_raw" and len(pts) < N_FRAMES:
            m = deserialize_message(data, PointCloud2)
            buf = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
            off = {f.name: f.offset for f in m.fields}
            cols = [buf[:, off[k]:off[k] + 4].copy().view("<f4").ravel().astype(np.float64)
                    for k in ("x", "y", "z")]
            pts.append(np.stack(cols, 1))
        elif topic == "/lidar/front/imu" and len(accs) < IMU_N:
            m = deserialize_message(data, Imu)
            accs.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        elif topic == "/tf_static":
            for t in deserialize_message(data, TFMessage).transforms:
                tfs.setdefault((t.header.frame_id, t.child_frame_id),
                               (np.array([t.transform.translation.x, t.transform.translation.y,
                                          t.transform.translation.z]),
                                rot(t.transform.rotation.x, t.transform.rotation.y,
                                    t.transform.rotation.z, t.transform.rotation.w)))

    P = np.concatenate(pts, 0)
    P = P[np.isfinite(P).all(1)]
    rad = np.linalg.norm(P, axis=1)
    P = P[(rad > 0.15) & (rad < 3.0)]
    Ps = P[::7]                       # 抽稀，RANSAC 够用
    print(f"点云: {len(pts)} 帧, 有效点 {len(P)}, 抽稀后 {len(Ps)}")

    rng = np.random.default_rng(0)
    cands = []
    for _ in range(4000):
        i = rng.choice(len(Ps), 3, replace=False)
        p1, p2, p3 = Ps[i]
        n = np.cross(p2 - p1, p3 - p1)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        d = float(n.dot(p1))
        if abs(d) < 0.2 or abs(d) > 0.8:
            continue
        inl = int((np.abs(Ps.dot(n) - d) < 0.03).sum())
        if inl < 0.02 * len(Ps):
            continue
        if all(ang(n, c[1]) > 2.0 or abs(abs(d) - abs(c[2])) > 0.05 for c in cands):
            cands.append((inl, n.copy(), d))
    cands.sort(key=lambda c: -c[0])
    A0 = np.array(accs).mean(0)
    A0 = A0 / np.linalg.norm(A0)
    print(f"候选大平面（|d| 0.2~0.8 m, 覆盖率 >=2%）：{len(cands)} 个，按内点数排序取前 5：")
    floor = None
    for k, (inl, n, d) in enumerate(cands[:5]):
        horiz = ang(n, A0)
        print(f"  #{k}: 内点 {inl:6d} ({100.0*inl/len(Ps):5.1f}%) d={d:+.4f} n={np.round(n,4)} "
              f"| angle(n, â)={horiz:5.1f}°")
        if floor is None and (horiz < 10.0 or horiz > 170.0):
            floor = (inl, n.copy(), d)          # 近水平的那个才是地面
    if floor is None:
        print("未找到近水平大平面（点云里没有地面？）")
        return
    n, d = floor[1], floor[2]
    if d < 0:                                   # 点在 +n 侧 ⇒ 传感器在另一侧
        n, d = -n, -d
    for _ in range(3):                          # SVD 精修
        m = np.abs(P.dot(n) - d) < 0.03
        Q = P[m]
        if len(Q) > 20000:
            Q = Q[:: max(1, len(Q) // 20000)]
        c = Q.mean(0)
        _, _, V = np.linalg.svd(Q - c, full_matrices=False)
        n = V[-1]
        if n.dot(c) < 0:
            n = -n
        d = float(n.dot(c))
    inl = int((np.abs(P.dot(n) - d) < 0.03).sum())
    up_floor = -n
    print(f"地面平面: 内点 {inl}/{len(P)} ({100.0*inl/len(P):.1f}%), |d| = {d:.4f} m (= 传感器离地高度)")
    print(f"  法向(指向地面) n = {np.round(n,4)}   上 = -n = {np.round(up_floor,4)}")
    print(f"  上 与各轴夹角: +x {ang(up_floor,[1,0,0]):.1f}° +y {ang(up_floor,[0,1,0]):.1f}° "
          f"+z {ang(up_floor,[0,0,1]):.1f}° -x {ang(up_floor,[-1,0,0]):.1f}°")

    A = np.array(accs)
    a_hat = A.mean(0)
    an = np.linalg.norm(a_hat)
    a_hat_u = a_hat / an
    print(f"IMU 首 {len(accs)} 采样均值 â = {np.round(a_hat,5)} |a| = {an:.4f}")
    print(f"  加计方向散布: 最大 {np.degrees(np.arccos(np.clip((A/an).dot(a_hat_u),-1,1))).max():.2f}°")
    print(f"  ★ angle(上_地面 , â) = {ang(up_floor, a_hat_u):.2f}°"
          f"   （≈0 ⇒ 加计朝上；≈180 ⇒ 加计朝下，需翻符号）")
    print(f"  ★ angle(上_地面 , -â) = {ang(up_floor, -a_hat_u):.2f}°")

    R_bl = np.eye(3)
    for a, b in (("base_link", "rslidartmp1"), ("rslidartmp1", "laser_points_front")):
        t, Rm = tfs[(a, b)]
        R_bl = R_bl.dot(Rm)
        print(f"TF {a}->{b}: t={np.round(t,4)}  R=\n{np.round(Rm,4)}")
    up_urdf = R_bl.T.dot(np.array([0.0, 0.0, 1.0]))
    print(f"URDF 链 up = R_base_laser^T·ẑ = {np.round(up_urdf,4)}")
    print(f"  angle(up_urdf, 上_地面) = {ang(up_urdf, up_floor):.2f}°   "
          f"angle(up_urdf, â) = {ang(up_urdf, a_hat_u):.2f}°   angle(up_urdf, -â) = {ang(up_urdf, -a_hat_u):.2f}°")


def rosbag2_conv(a, b):
    return rosbag2_py.ConverterOptions(a, b)


if __name__ == "__main__":
    main()
