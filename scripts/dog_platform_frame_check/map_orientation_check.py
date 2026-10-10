#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""★★判"世界系 z 是否物理朝上"（2026-10-09 第7次修复的判据；原 /tmp/e1/orient.py 归档）★★

原理：世界系 z 由 IMUInit() 对齐到 sign·â（believed up）；地面是物理上的"下方"。
      若地面在 z<0 ⇒ 世界系 z 朝上 ✓；若地面在 z>0 ⇒ 世界系 z 朝下（整图上下颠倒）✗。
做法：起点(=原点，ESKF 初位姿 pos=0)的 XY 半径 3 m 内取点，对 z∈[-1.5,1.5] 做 5 cm 直方图，
      取点数最多的 bin 作为地面（地面是这里最密的水平面），报"传感器离地 = -z_floor"。
      ⇒ 本平台正常应 ≈ +0.52 m（= 雷达离地高），负值说明整图上下颠倒。
用法：python3 map_orientation_check.py [global.pcd ...]   # 不给参数则用仓库 data/new_map/global.pcd
配套：doc/modification_log.md §4.2（第7次修复）、doc/dog_platform_frame_fix_report.md §0（勘误）。
"""
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'scripts', 'global_map_fix_check'))
from airy_save_diag import read_pcd_xyz  # noqa: E402

R_XY = 3.0     # 起点邻域半径 (m)
Z_LO, Z_HI, BIN = -1.5, 1.5, 0.05


def main():
    paths = sys.argv[1:] or [os.path.join(REPO, 'data', 'new_map', 'global.pcd')]
    print('%-14s %8s %9s %9s %9s %8s  %s' %
          ('变体', '点数', 'z_min', 'z_max', 'z_floor', '离地m', '结论'))
    for p in paths:
        d = os.path.dirname(p)
        for _ in range(2):
            d = os.path.dirname(d)
        tag = os.path.basename(d) or p
        if not os.path.exists(p):
            print('%-14s (文件不存在: %s)' % (tag, p))
            continue
        P = read_pcd_xyz(p)
        if P is None or len(P) == 0:
            print('%-14s (读不到 %s)' % (tag, p))
            continue
        r = np.linalg.norm(P[:, :2], axis=1)
        Q = P[r < R_XY]
        if len(Q) < 100:
            Q = P          # 起点附近没点（地图不在起点附近）⇒ 退回全图
        z = Q[:, 2]
        m = (z > Z_LO) & (z < Z_HI)
        h, e = np.histogram(z[m], bins=np.arange(Z_LO, Z_HI + BIN, BIN))
        zf = 0.5 * (e[h.argmax()] + e[h.argmax() + 1])
        hgt = -zf
        v = 'z 朝上 ✓' if hgt > 0.15 else ('上下颠倒 ✗' if hgt < -0.15 else '不确定')
        print('%-14s %8d %9.3f %9.3f %9.3f %8.3f  %s' %
              (tag, len(P), P[:, 2].min(), P[:, 2].max(), zf, hgt, v))


if __name__ == '__main__':
    main()
