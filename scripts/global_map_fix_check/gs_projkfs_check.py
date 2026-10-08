#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位输入端 ProjectKFs（投影历史关键帧点云）外参补丁的量化验证。

背景：`LaserMapping::ProjectKFs()` 把【历史关键帧的雷达系点云】投影到【当前帧雷达系】下，
      再与当前帧点云拼成定位端 LidarLoc 的输入（localization.cpp: lio_->GetProjCloud()）。
      补丁前它用的是"当前 IMU 系 ← 历史 IMU 系"的相对位姿直接乘雷达系点，漏了外参。

验证方法（无需任何真值地图）：投影结果与"当前帧实测点云"都在当前帧雷达系下，
      静态场景里两者应当互相重合，因此用最近邻距离即可判定：
          FIX = T_imu_lidar⁻¹ * (T_imu_cur_w * T_w_imu_kf) * T_imu_lidar
          BUG = T_imu_cur_w * T_w_imu_kf                                  (补丁前写法)

用法:
  python3 gs_projkfs_check.py [--log ..] [--yaml ..] [--bag ..] [--gap 25] [--samples 20]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_ab_map import (  # noqa: E402
    Rx, Ry, Rz, SE3, load_raw_frames, parse_extrinsic, parse_log, voxel,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default='/tmp/g2p5fix_run/stdout.log')
    ap.add_argument('--yaml', default='/tmp/g2p5fix_run/airy96_final.yaml')
    ap.add_argument('--bag', default='/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07')
    ap.add_argument('--topic', default='/lidar/front/points_raw')
    ap.add_argument('--gap', type=int, default=25, help='历史帧与当前帧的间隔（帧）')
    ap.add_argument('--samples', type=int, default=20)
    a = ap.parse_args()

    from scipy.spatial import cKDTree

    pos, rpy, M = parse_log(a.log)
    t_ext, R_ext = parse_extrinsic(a.yaml)
    T_imu_lidar = SE3(R_ext, t_ext)
    T_lidar_imu = np.linalg.inv(T_imu_lidar)
    M4 = SE3(M, np.zeros(3))
    frames = load_raw_frames(a.bag, a.topic, len(pos) + 8)
    print('日志帧数=%d, bag 帧数=%d, gap=%d 帧' % (len(pos), len(frames), a.gap))
    print('T_imu_lidar: t = %s\n R =\n%s' % (t_ext, np.round(R_ext, 4)))

    def Tw(i):
        return SE3(Rz(rpy[i][2]) @ Ry(rpy[i][1]) @ Rx(rpy[i][0]), pos[i]) @ M4

    res = {'FIX': [], 'BUG': [], 'gap_m': []}
    for i in np.linspace(a.gap, min(len(pos), len(frames)) - 1, a.samples).astype(int):
        j = i - a.gap
        if j < 0:
            continue
        Q = voxel(frames[j], leaf=0.2)
        if len(Q) < 100:
            continue
        Qh = np.c_[Q, np.ones(len(Q))].T

        T_rel = np.linalg.inv(Tw(i)) @ Tw(j)          # T_imu_cur_imu_kf
        P_fix = (T_lidar_imu @ T_rel @ T_imu_lidar @ Qh).T[:, :3]
        P_bug = (T_rel @ Qh).T[:, :3]

        tgt = frames[i]
        if len(tgt) < 100:
            continue
        tree = cKDTree(tgt)

        d_fix, _ = tree.query(P_fix, k=1, workers=-1)
        d_bug, _ = tree.query(P_bug, k=1, workers=-1)
        res['FIX'].append(np.median(d_fix))
        res['BUG'].append(np.median(d_bug))
        res['gap_m'].append(np.linalg.norm(pos[i] - pos[j]))

    f = np.array(res['FIX'])
    b = np.array(res['BUG'])
    print('样本数 = %d, 历史帧平均间距 = %.2f m (%.1f s)' %
          (len(f), np.mean(res['gap_m']), a.gap / 30.0))
    print('  投影点云 -> 当前帧实测点云 的 NN 中位距离:')
    print('    FIX（补丁后）: 中位=%.3f m, 均值=%.3f m, max=%.3f m' % (np.median(f), f.mean(), f.max()))
    print('    BUG（补丁前）: 中位=%.3f m, 均值=%.3f m, max=%.3f m' % (np.median(b), b.mean(), b.max()))
    print('  ★FIX/BUG = %.3f 倍' % (np.median(f) / max(np.median(b), 1e-9)))


if __name__ == '__main__':
    main()
