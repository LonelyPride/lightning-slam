#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""密度无关的“地面叠层”指标。

思路：把点云按 XY 切成 2x2 m 网格（每格点数与整体密度成比例地缩放，但判据用“层”而不是点），
在每格内沿 z 排序，按 >0.30 m 的间隙聚类。地面网格格应当只有 1 层（最底层）；
若地图把多帧错位的地面叠在一起（BUG 的“多层壳”伪影），同一格会出现 2 层以上。

输出：地面格的层数分布 / 多层格占比 / 最底层厚度 —— 与采样密度无关，只看结构层数。
"""
import sys
import numpy as np

import os  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_save_diag import read_pcd_xyz

CELL = 2.0      # XY 网格边长 (m)
GAP = 0.30      # z 方向层间间隙阈值 (m)
MIN_LOW = 15    # 最底层至少这么多点才算“地面格”
MIN_CELL = 30   # 该格总点数下限

MAPS = [("打补丁后 新保存 (FIX)", '/tmp/gs_fix_run/data/new_map/global.pcd'),
        ("打补丁前 旧保存 (BUG)", '/tmp/planAplus_run/data/new_map/global.pcd')]


def layers_of(z):
    """把一个格内的 z 数组切成若干层，返回每层的 (点数, 厚度)。"""
    z = np.sort(z)
    cuts = np.where(np.diff(z) > GAP)[0] + 1
    segs = np.split(z, cuts)
    return [(len(s), float(s[-1] - s[0])) for s in segs if len(s) > 0]


def analyze(path):
    P = read_pcd_xyz(path)
    P = P[np.isfinite(P).all(1)]
    ij = np.floor(P[:, :2] / CELL).astype(np.int64)
    key = ij[:, 0] * 1000003 + ij[:, 1]
    uniq, inv = np.unique(key, return_inverse=True)
    n_cell = n_ground = 0
    clust, thick, multi = [], [], 0
    for c in range(len(uniq)):
        z = P[inv == c, 2]
        if len(z) < MIN_CELL:
            continue
        n_cell += 1
        ls = layers_of(z)
        if not ls or ls[0][0] < MIN_LOW:
            continue
        n_ground += 1
        clust.append(len(ls))
        thick.append(ls[0][1])
        if len(ls) >= 2:
            multi += 1
    clust = np.asarray(clust); thick = np.asarray(thick)
    return dict(n_pts=len(P), n_cell=n_cell, n_ground=n_ground, clust=clust,
                thick=thick, multi=multi)


print("=== 地面叠层指标（XY %gx%g m 网格 / z 间隙 %.2f m 为一层）===" % (CELL, CELL, GAP))
for name, p in MAPS:
    try:
        r = analyze(p)
    except Exception as e:
        print("  %s 读取失败: %s" % (name, e)); continue
    c, t = r['clust'], r['thick']
    print("-" * 78)
    print("%s  点数=%d  有效网格=%d  地面格=%d" % (name, r['n_pts'], r['n_cell'], r['n_ground']))
    if len(c) == 0:
        print("   无地面格"); continue
    print("   每格层数: 均值=%.2f 中位=%.0f 分布(1层/2层/3层/>=4层) = %d / %d / %d / %d"
          % (c.mean(), np.median(c), int((c == 1).sum()), int((c == 2).sum()),
             int((c == 3).sum()), int((c >= 4).sum())))
    print("   多层格(>=2 层)占比 = %.1f%%   (地面被叠加成壳的比例)" % (100.0 * r['multi'] / len(c)))
    print("   最底层厚度: 中位=%.3f m  p90=%.3f m" % (np.median(t), np.percentile(t, 90)))
