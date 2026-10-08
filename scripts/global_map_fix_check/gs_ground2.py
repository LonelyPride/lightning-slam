#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""与密度无关、物理直观的"地面是否成面"指标。

做法：把地图投影到 XY，按 5x5 m 网格分格；每格用 z 的 2% 分位数作为该格的
"地面高度"（对离群点鲁棒，且每格只贡献 1 个数，故与点数密度无关）；
再对全部格子的地面高度做整体平面 z = a*x + b*y + c 的稳健拟合。
  * 拼接正确：同一片地面在所有方位都落在同一高度 -> 残差 ~0.0x m；
  * 漏外参  ：地面被"摊开"成多层壳，不同方位的地面高度差达米级 -> 残差 ~米级。
"""
import sys
import numpy as np

import os  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_save_diag import read_pcd_xyz

CELL = 5.0


def fit_plane_robust(XY, Z):
    A = np.c_[XY[:, 0], XY[:, 1], np.ones(len(XY))]
    keep = np.ones(len(XY), dtype=bool)
    coef = np.zeros(3)
    for _ in range(5):
        coef, *_ = np.linalg.lstsq(A[keep], Z[keep], rcond=None)
        r = Z - A @ coef
        s = np.median(np.abs(r - np.median(r))) * 1.4826 + 1e-9
        keep = np.abs(r - np.median(r)) < 3.0 * s
    r = Z - A @ coef
    return coef, r, keep


def ground_flatness(P, tag):
    gx = np.floor((P[:, 0] - P[:, 0].min()) / CELL).astype(np.int64)
    gy = np.floor((P[:, 1] - P[:, 1].min()) / CELL).astype(np.int64)
    key = gx * 100000 + gy
    XY, Z = [], []
    for k in np.unique(key):
        Q = P[key == k]
        if len(Q) < 200:
            continue
        XY.append([Q[:, 0].mean(), Q[:, 1].mean()])
        Z.append(np.percentile(Q[:, 2], 2))
    XY = np.array(XY)
    Z = np.array(Z)
    coef, r, keep = fit_plane_robust(XY, Z)
    rk = r[keep]
    print("%-16s 网格数=%3d  地面 z 跨度(格间)=%6.2f m | 拟合全局平面残差 RMS=%.3f  中位=%.3f  p90=%.3f m"
          % (tag, len(XY), np.ptp(Z), np.sqrt((rk ** 2).mean()), np.median(np.abs(rk)),
             np.percentile(np.abs(rk), 90)))
    return float(np.sqrt((rk ** 2).mean()))


if __name__ == '__main__':
    print("=== 每 5x5 m 网格地面高度 -> 拟合一张全局地面平面的残差（与点密度无关）===")
    a = ground_flatness(read_pcd_xyz('/tmp/gs_fix_run/data/new_map/global.pcd'), "打补丁后 新保存")
    b = ground_flatness(read_pcd_xyz('/tmp/planAplus_run/data/new_map/global.pcd'), "打补丁前 旧保存")
    print("地面整体平面度: 旧(BUG) %.3f m -> 新(补丁后) %.3f m   改善 %.1f 倍" % (b, a, b / a if a > 0 else 0))
