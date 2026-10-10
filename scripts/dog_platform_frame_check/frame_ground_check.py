#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""★★第6次修复的最终验收（全部量都锚定在 bag / tf_static 上，不依赖 URDF 文本）★★

A) 从 bag 的 /tf_static 自行串出 base_link→…→laser_points_front 的链，算"车体上"在雷达(=IMU)系的表达
   up_laser = R_base_laserᵀ · ẑ；再取 bag 前 233 个 IMU 采样（= 本次运行的静止初始化窗口）的比力均值 â。
   → angle(up_laser, â) 应 ≈180°（加计比力朝下 ⇒ 必须 imu_acc_up_sign=-1），angle(up_laser, -â) 应 ≈0。
B) 两份地图在【起点】处的地面：RANSAC 拟合起点周围 3 m 的点云平面，
   报"地面相对起点传感器的高度"（应 ≈ -0.46 m = g2p5.lidar_height）与法向相对竖直的夹角。
配套报告：doc/dog_platform_frame_fix_report.md（本脚本原为 /tmp/final_check6.py，原样归档）
数据路径：改下面的 BAG / NEW_MAP / OLD_MAP / OLD_LOG 常量即可。
"""
import math
import os
import re
import sys

import numpy as np

BAG = '/home/h/ddh/bag/dog/20261009/短途/jieya/rosbag2_2026_10_09-10_50_41'
# 公共依赖在 ../global_map_fix_check/（用相对本文件的路径，整仓库搬走也能跑）
SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'global_map_fix_check')
sys.path.insert(0, SCRIPT_DIR)
from airy_ab_map import read_pcd_xyz  # noqa: E402

NEW_MAP = '/home/h/ddh/code/ws/lightning_ws/data/new_map'
OLD_MAP = '/tmp/old_map_before_fix/data/new_map'


def ang(u, v):
    u = np.asarray(u, float); v = np.asarray(v, float)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(u.dot(v) / (np.linalg.norm(u) * np.linalg.norm(v)))))))


def q2R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def read_tf_static_and_accel():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    edges, acc, n = {}, [], 0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/tf_static':
            for t in deserialize_message(data, TFMessage).transforms:
                edges[t.child_frame_id] = (t.header.frame_id,
                                           np.array([t.transform.translation.x, t.transform.translation.y,
                                                     t.transform.translation.z]),
                                           q2R([t.transform.rotation.x, t.transform.rotation.y,
                                                t.transform.rotation.z, t.transform.rotation.w]))
        elif topic == '/lidar/front/imu' and n < 233:
            m = deserialize_message(data, Imu)
            acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
            n += 1
        if edges and n >= 233:
            break
    return edges, np.array(acc).mean(axis=0)


def chain_up(edges, parent, child):
    """返回 (t, R)：T_parent_child（沿链组合）。"""
    t = np.zeros(3); R = np.eye(3)
    cur = child
    while cur != parent:
        p, tt, RR = edges[cur]
        t = tt + RR @ t
        R = RR @ R
        cur = p
    return t, R


def floor_at(P, xy, radius=3.0, band=0.02, iters=400, seed=0):
    """RANSAC 拟合 (x,y) 附近 radius 内的平面 → (平面 z 中位, 法向, 内点数, 采样点数)"""
    d = np.linalg.norm(P[:, :2] - xy, axis=1)
    Q = P[d < radius]
    if len(Q) < 100:
        return None
    rs = np.random.RandomState(seed)
    best = None
    for _ in range(iters):
        i = rs.choice(len(Q), 3, replace=False)
        a, b, c = Q[i]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        dist = np.abs((Q - a) @ n)
        cnt = int((dist < band).sum())
        if best is None or cnt > best[0]:
            best = (cnt, n, a, dist)
    cnt, n, a, dist = best
    if n[2] < 0:
        n = -n
    return float(np.median(Q[dist < band][:, 2])), n, cnt, len(Q)


def start_pose(map_dir):
    txt = open(map_dir + '/index.txt', errors='ignore').read()
    m = re.search(r'start\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)', txt)
    return np.array([float(m.group(1)), float(m.group(2)), float(m.group(3))])


OLD_LOG = '/tmp/run_slam_online.h-TX-Gaming-FX608JMR-FX608JMR.h.log.INFO.20261009-104117.18276'

if __name__ == '__main__':
    edges, acc_mean = read_tf_static_and_accel()
    print('=== A) bag /tf_static 链 + 加计静止均值（前 233 采样）===')
    for c, (p, t, q) in sorted(edges.items()):
        print('   %-22s <- %-22s' % (c, p))
    t_bl, R_bl = chain_up(edges, 'base_link', 'laser_points_front')
    up_laser = R_bl.T @ np.array([0.0, 0.0, 1.0])          # 车体"上"在雷达系
    print('   T_base_laser: t=%s (|t|=%.3f m)' % (np.round(t_bl, 4), np.linalg.norm(t_bl)))
    print('   车体"上"在雷达系 up_laser = R_base_laser^T·ẑ = %s' % np.round(up_laser, 4))
    print('   加计静止均值 â = %s (|â|=%.4f g)' % (np.round(acc_mean, 4), np.linalg.norm(acc_mean)))
    print('   angle(up_laser,  â) = %6.2f°  <-- ≈180° ⇒ 加计比力朝【下】，必须 imu_acc_up_sign: -1'
          % ang(up_laser, acc_mean))
    print('   angle(up_laser, -â) = %6.2f°  <-- 应 ≈0（车体"上" = 加计读数取反）'
          % ang(up_laser, -acc_mean))

    print()
    print('=== A2) 运行日志里的 R_vehicle_imu 第3行（车体"上"在 IMU 系）与 up_laser 对比 ===')
    for tag, log in (('新(第6次修复后)', '/tmp/run6_offline.log'), ('旧(修复前)', OLD_LOG)):
        lines = open(log, errors='ignore').read().splitlines()
        for i, l in enumerate(lines):
            if 'R_vehicle_imu =' in l:
                row = np.array([float(v) for v in
                                re.findall(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?', lines[i + 3])][:3])
                a = ang(row, up_laser)
                print('   %s R_vehicle_imu[2] = %s ⇒ 与 up_laser 相差 %6.2f°  %s'
                      % (tag, np.round(row, 4), a, '✓ 车体没被装反' if a < 5 else '✗ 差 180° = 车体被装反'))
                break

    print()
    print('=== B) 起点处地面（RANSAC，起点周围 3 m）与传感器高度 ===')
    for tag, d in (('新地图(12:09)', NEW_MAP), ('旧地图(10:42)', OLD_MAP)):
        P = read_pcd_xyz(d + '/global.pcd').astype(np.float64)
        s = start_pose(d)
        res = floor_at(P, s[:2])
        if res is None:
            print('   %s: 起点附近点太少' % tag)
            continue
        z, n, cnt, tot = res
        print('   %s 起点=%s 地图点数=%d' % (tag, np.round(s, 3), len(P)))
        print('      地面 z = %.3f m ⇒ 传感器离地 = %.3f m（g2p5.lidar_height 期望 0.46）' % (z, s[2] - z))
        print('      地面法向 = %s，与竖直夹角 = %.2f°（RANSAC 内点 %d/%d）'
              % (np.round(n, 4), ang(n, [0, 0, 1]), cnt, tot))

