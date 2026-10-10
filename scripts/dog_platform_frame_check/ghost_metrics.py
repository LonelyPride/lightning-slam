#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""★★"重影/叠层"量化脚本（2026-10-09 第7次修复的判据；原 /tmp/e1/metrics.py 归档）★★

为什么需要它：第6次修复把 extrinsic_R 写成单位阵后，地图**看着正常**（世界系 z 朝上、地面高度也对），
  但同一物理表面被多帧按略微不同的位置叠上去 = 【重影】。只目视/只看"地图像不像"抓不到，
  必须量"表面厚度"和"双面占比"。本次实测：单位阵 115 mm / 8.9%，正确外参 50.7 mm / 0.7%。

对每份 global.pcd 计算：
  ① 表面厚度 : 固定半径 r 邻域 PCA 最小特征值开方（mm），中位 / p90
  ② 双面占比 : 邻域点投影到局部法向后做"间隙 > GAP 的聚类"，若存在 ≥2 个质量各 ≥MASS 的簇
                ⇒ 记为"双面/重影"（地板、墙、天花板都适用）
  ③ 地面叠层 : 工程自带 gs_layers.py 的思路（XY 2x2 m 网格内沿 z 按 LAYER_GAP 分层）
用法：python3 ghost_metrics.py [global.pcd ...]   # 不给参数则用仓库 data/new_map/global.pcd
      多个路径可一次比较（表里按 basename 区分；多份同名时看顺序）。
配套：doc/modification_log.md §4.2（第7次修复）、doc/dog_platform_frame_fix_report.md §0（勘误）。
"""
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'scripts', 'global_map_fix_check'))
from airy_save_diag import read_pcd_xyz  # noqa: E402

R_NEI = 0.6        # 邻域半径 (m)
N_SAMPLE = 1500    # 采样点
GAP = 0.03         # 法向方向"双面"判定间隙 (m)
MASS = 0.15        # 每个面的最小质量占比
CELL, LAYER_GAP, MIN_LOW, MIN_CELL = 2.0, 0.30, 15, 30


def ground_layers(P):
    """工程 gs_layers.py 的指标：地面格内 z 分层数。"""
    ij = np.floor(P[:, :2] / CELL).astype(np.int64)
    key = ij[:, 0] * 1000003 + ij[:, 1]
    uniq, inv = np.unique(key, return_inverse=True)
    clust = []
    for c in range(len(uniq)):
        z = np.sort(P[inv == c, 2])
        if len(z) < MIN_CELL:
            continue
        cuts = np.where(np.diff(z) > LAYER_GAP)[0] + 1
        segs = np.split(z, cuts)
        if len(segs[0]) < MIN_LOW:
            continue
        clust.append(len(segs))
    return np.asarray(clust)


def analyze(path, tag):
    P = read_pcd_xyz(path).astype(np.float64)
    P = P[np.isfinite(P).all(1)]
    if len(P) < 100:
        print('  %-12s 点数过少' % tag)
        return None
    tree = cKDTree(P)
    nn = tree.query(P, k=2, workers=-1)[0][:, 1]
    rs = np.random.RandomState(0)
    idx = rs.choice(len(P), min(N_SAMPLE, len(P)), replace=False)

    thick, multi = [], []
    floor_m, wall_m, floor_n, wall_n = 0, 0, 0, 0
    for i in idx:
        nb = tree.query_ball_point(P[i], R_NEI)
        if len(nb) < 15:
            continue
        Q = P[nb] - P[nb].mean(axis=0, keepdims=True)
        w, v = np.linalg.eigh(Q.T @ Q / len(nb))
        nv = v[:, 0]
        thick.append(np.sqrt(max(w[0], 0.0)))
        d = np.sort(Q @ nv)
        cuts = np.where(np.diff(d) > GAP)[0] + 1
        segs = np.split(d, cuts)
        big = sum(1 for s in segs if len(s) >= MASS * len(d))
        m = 1 if big >= 2 else 0
        multi.append(m)
        if abs(nv[2]) > 0.8:
            floor_n += 1
            floor_m += m
        elif abs(nv[2]) < 0.3:
            wall_n += 1
            wall_m += m
    thick = np.asarray(thick)
    multi = np.asarray(multi)
    gl = ground_layers(P)
    return dict(tag=tag, n=len(P), nn=float(np.median(nn)),
                th_med=1e3 * float(np.median(thick)),
                th_p90=1e3 * float(np.percentile(thick, 90)),
                mf=100.0 * float(multi.mean()),
                mf_floor=100.0 * float(floor_m / max(floor_n, 1)),
                mf_wall=100.0 * float(wall_m / max(wall_n, 1)),
                nv=len(thick),
                lay=float(np.mean(gl)) if len(gl) else float('nan'),
                lay_frac=100.0 * float((gl >= 2).mean()) if len(gl) else float('nan'),
                zmin=float(P[:, 2].min()), zmax=float(P[:, 2].max()))


def main(paths):
    rows = []
    for tag, p in paths:
        if not os.path.exists(p):
            print('  %-12s 文件不存在: %s' % (tag, p))
            continue
        r = analyze(p, tag)
        if r:
            rows.append(r)
    print()
    print('%-12s %8s %7s %9s %9s %8s %9s %9s %8s %8s %8s' %
          ('变体', '点数', 'NN中位m', '厚度中位mm', '厚度p90mm', '双面%', '地板双面%', '墙双面%',
           '叠层均', '多层格%', 'z跨度'))
    for r in rows:
        print('%-12s %8d %7.3f %9.1f %9.1f %8.1f %9.1f %9.1f %8.2f %8.1f %8.1f' %
              (r['tag'], r['n'], r['nn'], r['th_med'], r['th_p90'], r['mf'], r['mf_floor'],
               r['mf_wall'], r['lay'], r['lay_frac'], r['zmax'] - r['zmin']))
    print('\n判读：正确外参 ≈ 厚度中位 50 mm / 双面 <1.5%；与单位阵（115 mm / 8.9%）差一倍以上。')


if __name__ == '__main__':
    def _tag(p):
        """标签取 global.pcd 往上第三级目录（形如 <变体>/data/new_map/global.pcd ⇒ <变体>）。"""
        d = os.path.dirname(p)
        for _ in range(2):
            d = os.path.dirname(d)
        return os.path.basename(d) or p

    if len(sys.argv) > 1:
        main([(_tag(p), p) for p in sys.argv[1:]])
    else:
        main([('data/new_map', os.path.join(REPO, 'data', 'new_map', 'global.pcd'))])
