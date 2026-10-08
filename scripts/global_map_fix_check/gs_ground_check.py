#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""与密度无关的直观指标：地图地面厚度（每个 5x5 m 网格内，最低点带的 z 向厚度）。
   正确拼接时地面应成面（厚度 ~0.05-0.2 m）；漏外参时地面被"摊开"（厚度 米级）。
"""
import sys
import numpy as np

import os  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_save_diag import read_pcd_xyz

CELL = 5.0
NBAND = 0.5


def ground_thickness(P, tag):
    gx = np.floor((P[:, 0] - P[:, 0].min()) / CELL).astype(np.int64)
    gy = np.floor((P[:, 1] - P[:, 1].min()) / CELL).astype(np.int64)
    key = gx * 100000 + gy
    th, ncell = [], 0
    for k in np.unique(key):
        Q = P[key == k]
        if len(Q) < 200:
            continue
        z0 = np.percentile(Q[:, 2], 5)
        band = Q[(Q[:, 2] >= z0) & (Q[:, 2] <= z0 + NBAND), 2]
        if len(band) < 50:
            continue
        th.append(float(np.ptp(band)))
        ncell += 1
    th = np.array(th)
    print("%-22s 网格数=%3d  地面厚度 中位=%.3f m  均值=%.3f  p90=%.3f  max=%.3f m"
          % (tag, ncell, np.median(th), th.mean(), np.percentile(th, 90), th.max()))
    return np.median(th)


if __name__ == '__main__':
    new = read_pcd_xyz('/tmp/gs_fix_run/data/new_map/global.pcd')
    old = read_pcd_xyz('/tmp/planAplus_run/data/new_map/global.pcd')
    m_new = ground_thickness(new, "打补丁后 新保存")
    m_old = ground_thickness(old, "打补丁前 旧保存")
    print("地面厚度改善: %.3f m -> %.3f m (缩小 %.1f 倍)" % (m_old, m_new, m_old / m_new))
