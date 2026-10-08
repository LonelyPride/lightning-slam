#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""与点密度无关的"地面是否成面"指标：固定半径 r 邻域内做 PCA，取最小特征值对应 RMS。
   同一物理体积内的几何被同等采样，故可在"新保存(6.7万点)/旧保存(22万点)/重建(百万点)"间公平比较。
"""
import sys
import numpy as np

import os  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_ab_map import (Rz, Ry, Rx, parse_log, parse_extrinsic, SE3, load_raw_frames,
                         voxel, read_pcd_xyz)
from scipy.spatial import cKDTree

R = 0.8
SAMPLE = 1200


def planarity_radius(P, tag, r=R, sample=SAMPLE, seed=0):
    rs = np.random.RandomState(seed)
    idx = rs.choice(len(P), min(sample, len(P)), replace=False)
    tree = cKDTree(P)
    nbrs = tree.query_ball_point(P[idx], r=r, workers=-1)
    sig = []
    for nb in nbrs:
        if len(nb) < 8:
            continue
        Q = P[nb].astype(np.float64)
        Q = Q - Q.mean(axis=0, keepdims=True)
        w = np.linalg.eigvalsh(Q.T @ Q / len(Q))
        sig.append(np.sqrt(max(w[0], 0.0)))
    sig = np.array(sig)
    print("  %-26s 有效采样=%4d  r=%.1f m 邻域内平面RMS 中位=%.3f p90=%.3f m"
          % (tag, len(sig), r, np.median(sig), np.percentile(sig, 90)))
    return float(np.median(sig))


if __name__ == '__main__':
    LOG, YAML = '/tmp/gs_fix_run/stdout.log', '/tmp/gs_fix_run/airy96_final.yaml'
    BAG = '/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07'
    pos, rpy, M = parse_log(LOG)
    t_ext, R_ext = parse_extrinsic(YAML)
    T_imu_lidar = SE3(R_ext, t_ext)
    M4 = SE3(M, np.zeros(3))
    frames = load_raw_frames(BAG, '/lidar/front/points_raw', len(pos) + 1)
    n = min(len(pos), len(frames))
    bug, fix = [], []
    for i in range(n):
        Tw_imu = SE3(Rz(rpy[i][2]) @ Ry(rpy[i][1]) @ Rx(rpy[i][0]), pos[i]) @ M4
        Q = voxel(frames[i], 0.2)
        Qh = np.c_[Q, np.ones(len(Q))].T
        bug.append((Tw_imu @ Qh).T[:, :3])
        fix.append((Tw_imu @ T_imu_lidar @ Qh).T[:, :3])
    bug = np.vstack(bug).astype(np.float32)
    fix = np.vstack(fix).astype(np.float32)
    new = read_pcd_xyz('/tmp/gs_fix_run/data/new_map/global.pcd')
    old = read_pcd_xyz('/tmp/planAplus_run/data/new_map/global.pcd')
    print("=== 固定半径 %.1f m 邻域平面度（越小越接近平面）===" % R)
    planarity_radius(new, "新保存(打补丁后)")
    planarity_radius(old, "旧保存(打补丁前)")
    planarity_radius(fix, "FIX 重建(正确数学)")
    planarity_radius(bug, "BUG 重建(漏外参)")
