#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""公平对照：把 BUG/FIX 重建图也降到与 save_map 相同的 0.2 m 体素，再比平面度/跨度。
   目的：确认"新保存地图平面度 0.029"完全由 0.2 m 保存分辨率解释，而非残余错位。
"""
import sys
import numpy as np

import os  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_ab_map import (Rz, Ry, Rx, parse_log, parse_extrinsic, SE3, load_raw_frames,
                         voxel, nn_stats, planarity, read_pcd_xyz)

LOG = '/tmp/gs_fix_run/stdout.log'
YAML = '/tmp/gs_fix_run/airy96_final.yaml'
BAG = '/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07'
NEW = '/tmp/gs_fix_run/data/new_map/global.pcd'

pos, rpy, M = parse_log(LOG)
t_ext, R_ext = parse_extrinsic(YAML)
T_imu_lidar = SE3(R_ext, t_ext)
M4 = SE3(M, np.zeros(3))
frames = load_raw_frames(BAG, '/lidar/front/points_raw', len(pos) + 1)
n = min(len(pos), len(frames))

for leaf in (0.2,):
    bug, fix = [], []
    for i in range(n):
        Tw_imu = SE3(Rz(rpy[i][2]) @ Ry(rpy[i][1]) @ Rx(rpy[i][0]), pos[i]) @ M4
        Q = voxel(frames[i], leaf)
        Qh = np.c_[Q, np.ones(len(Q))].T
        bug.append((Tw_imu @ Qh).T[:, :3])
        fix.append((Tw_imu @ T_imu_lidar @ Qh).T[:, :3])
    bug = np.vstack(bug).astype(np.float32)
    fix = np.vstack(fix).astype(np.float32)
    new = read_pcd_xyz(NEW)
    print("=== 保存分辨率 leaf=%.1f m（save_map_resolution），%d 帧 ===" % (leaf, n))
    for tag, P in (('新保存(fixed)', new), ('BUG 重建', bug), ('FIX 重建', fix)):
        m, p90 = planarity(P)
        print("  %-14s 点数=%8d z跨度=%5.1f m 平面度RMS 中位=%.3f p90=%.3f m"
              % (tag, len(P), float(np.ptp(P[:, 2])), m, p90))
    for tag, P in (('BUG 重建', bug), ('FIX 重建', fix)):
        mean, med, p95 = nn_stats(new, P)
        print("  新保存 -> %-8s NN mean=%.3f median=%.3f p95=%.3f m" % (tag, mean, med, p95))
