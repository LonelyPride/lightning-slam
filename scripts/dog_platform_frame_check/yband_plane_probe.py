#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查清 vehicle_tilt_and_ground_check.py 报的"离地高 0.125 m、法向≈+y"那个面到底是什么。

做法：取同一 bag 的前 6 帧 /lidar/front/points_raw，在雷达系里
  ① 用 RANSAC 得到的地面（n=(0.6699,0.6601,0.3399)、d=0.4862）作参照；
  ② 选出 |p_y - 0.125| < 0.05 的点（即该脚本直方图尖峰那条"带"），
     看它们在【地面平面】上是不是正好躺着（p·n - d ≈ 0）⇒ 是地面被"薄带"切出来的一段，
     还是一片真的垂直于 +y 的独立表面（p·n - d 会明显不为 0）。
另外给出该带点的跨度/形状（长×宽），说明 SVD 拟合为什么病态。

配套报告：doc/dog_platform_frame_fix_report.md §6 第 6 条
（本脚本原为 /tmp/qb_plane_probe.py，原样归档）
"""
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

BAG = "/home/h/ddh/bag/dog/20261009/短途/jieya/rosbag2_2026_10_09-10_50_41"
N_FRAMES = 6
N_GROUND = np.array([0.6699, 0.6601, 0.3399])   # 指向地面（probe12 的 RANSAC 结果）
D_GROUND = 0.4862


def ang(a, b):
    a = np.asarray(a, float) / np.linalg.norm(a)
    b = np.asarray(b, float) / np.linalg.norm(b)
    return np.degrees(np.arccos(np.clip(a.dot(b), -1.0, 1.0)))


def main():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    pts = []
    while r.has_next() and len(pts) < N_FRAMES:
        topic, data, _ = r.read_next()
        if topic != "/lidar/front/points_raw":
            continue
        m = deserialize_message(data, PointCloud2)
        buf = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
        off = {f.name: f.offset for f in m.fields}
        pts.append(np.stack([buf[:, off[k]:off[k] + 4].copy().view("<f4").ravel().astype(np.float64)
                             for k in ("x", "y", "z")], 1))
    P = np.concatenate(pts, 0)
    P = P[np.isfinite(P).all(1)]
    print("点云 %d 帧, 有效点 %d" % (len(pts), len(P)))

    band = P[np.abs(P[:, 1] - 0.125) < 0.05]
    print("\n【该脚本尖峰处的点】|y-0.125|<0.05 : %d 点 (%.1f%% of 全部)"
          % (len(band), 100.0 * len(band) / len(P)))
    if len(band) < 10:
        return
    dist = band.dot(N_GROUND) - D_GROUND        # 相对"地面平面"的带符号距离
    print("  这些点到【地面平面】的距离: 中位 %+.4f m, 中位绝对 %.4f m, |d|<0.03 占 %.1f%%"
          % (np.median(dist), np.median(np.abs(dist)), 100.0 * (np.abs(dist) < 0.03).mean()))
    print("  ⇒ 若≈0 且占比高：这批点本来就躺在地面上（是地面被 y 薄带切出的'条带'），不是另一块面。")
    # 形状：在地面平面内建立 2D 坐标，看长宽
    e1 = np.cross(N_GROUND, [0.0, 0.0, 1.0]); e1 /= np.linalg.norm(e1)
    e2 = np.cross(N_GROUND, e1)
    u = band.dot(e1); v = band.dot(e2)
    print("  条带在地面内的尺寸: u 跨度 %.2f m, v 跨度 %.2f m (宽/长比 %.3f ⇒ SVD 病态)"
          % (u.max() - u.min(), v.max() - v.min(), (u.max() - u.min()) / max(1e-9, v.max() - v.min())))
    # 该带点自己 SVD 拟合出来的法向（复现脚本现象）
    c = band.mean(0)
    _, _, V = np.linalg.svd(band - c, full_matrices=False)
    n_fit = V[-1]
    if n_fit.dot(c) < 0:
        n_fit = -n_fit
    print("  只用这批点做 SVD 拟合 ⇒ 拟合法向 = %s" % np.round(n_fit, 4))
    print("     与雷达 +y 夹角 %.2f°,  与真实地面法向夹角 %.2f°"
          % (ang(n_fit, [0, 1, 0]), ang(n_fit, N_GROUND)))
    print("  ⇒ 现象复现：薄条带上做 SVD，法向被拉向 +y ⇒ 脚本的'地面法向/离地高'在本平台不可用。")


if __name__ == "__main__":
    main()
