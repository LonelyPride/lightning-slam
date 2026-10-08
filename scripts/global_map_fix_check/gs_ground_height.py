#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从"已修复的 global.pcd + 轨迹"测量：世界系下地面是否水平、雷达离地高度 h。

用途：给 yaml 的 g2p5.lidar_height / floor_height 提供实测依据
     （换算到车辆系后地面在传感器下方 h 处 => lidar_height = h, floor_height = -h）
"""
import os
import sys

import numpy as np

HERE = '/home/h/ddh/code/ws/lightning_ws/src/lightning-slam/scripts/global_map_fix_check'
sys.path.insert(0, HERE)
from airy_ab_map import parse_log, read_pcd_xyz  # noqa: E402

LOG = '/tmp/gs_fix_run/stdout.log'
PCD = '/tmp/gs_fix_run/data/new_map/global.pcd'


def main():
    pos, rpy, M = parse_log(LOG)
    P = read_pcd_xyz(PCD).astype(np.float64)
    print('轨迹帧数=%d, 地图点数=%d' % (len(pos), len(P)))
    print('世界系 z 范围: [%.2f, %.2f]' % (P[:, 2].min(), P[:, 2].max()))

    hs, nzs, gaps = [], [], []
    idx = np.linspace(0, len(pos) - 1, 120).astype(int)
    for i in idx:
        p = pos[i]
        d = np.linalg.norm(P[:, :2] - p[:2], axis=1)
        q = P[d < 8.0]
        if len(q) < 200:
            continue
        # 取局部最低的 20%（近似地面）做平面拟合
        z = q[:, 2]
        thr = np.percentile(z, 20)
        g = q[z < thr]
        if len(g) < 50:
            continue
        c = g.mean(axis=0)
        A = g - c
        w, v = np.linalg.eigh(A.T @ A)
        n = v[:, 0]                       # 最小特征值方向 = 平面法向
        if n[2] < 0:
            n = -n
        nzs.append(n[2])
        hs.append(p[2] - c[2])
        gaps.append(p[2] - np.percentile(z, 5))

    hs = np.array(hs)
    nzs = np.array(nzs)
    print('参与统计的采样点 = %d' % len(hs))
    print('地面法向 z 分量: 中位=%.4f, 5%%~95%%=[%.3f, %.3f]  (接近 1 说明世界系确实是 z 向上/地面水平)'
          % (np.median(nzs), np.percentile(nzs, 5), np.percentile(nzs, 95)))
    print('雷达离地高度 h(轨迹z - 局部地面z): 中位=%.3f m, 5%%~95%%=[%.3f, %.3f] m'
          % (np.median(hs), np.percentile(hs, 5), np.percentile(hs, 95)))
    print('冗余算法(轨迹z - 局部 z 的 5%% 分位): 中位=%.3f m' % np.median(gaps))


if __name__ == '__main__':
    main()
