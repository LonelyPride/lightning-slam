#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""g2p5 栅格地图(map.pgm) 与"已复核点云(global.pcd)"的一致性检查 —— 与点密度无关。

判据（同一目录下的产物天然同源、同世界系，可直接比较）：
  1) 栅格范围：g2p5 的 ResizeMap 只按"点云 XY 外扩 1 m"扩图，
     所以栅格 XY 范围应当 ≈ 该次运行 global.pcd 的 XY 包围盒 + 1 m；
     坐标系错位（漏外参/用错坐标系）时栅格会明显"外溢"。
  2) 占用格与点云的一致性：每个占用格(黑)在 XY 上到最近点云点的距离，
     以及占用格中"附近 0.3 m 内确有点云"的占比。
  3) 完备性：点云中"高于地面 0.3~1.8 m"的障碍点里，有多少在 0.3 m 内能找到占用格。

用法:
  python3 gs_g2p5_check.py [--run DIR] [--tag NAME] [--render OUT.png]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_ab_map import parse_log, read_pcd_xyz  # noqa: E402

RES = 0.05          # map.yaml 里写死的分辨率(slam.cc)
LIDAR_H = 1.81      # 实测雷达离地高度(见 gs_ground_height.py)


def read_pgm(path):
    """读 P5 PGM -> (H, W) uint8。约定：0=占用(黑), 255=空闲(白), 128=未知(灰)"""
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        assert magic == b'P5', magic
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = [int(x) for x in line.split()]
        maxv = int(f.readline())
        assert maxv == 255
        buf = f.read(w * h)
    return np.frombuffer(buf, dtype=np.uint8).reshape(h, w), w, h


def occupied_world_xy(img, w, h, origin, step=1):
    """占用格 -> 世界 XY（ROS 约定：图像左下角为 origin，y 向上）"""
    rows, cols = np.nonzero(img == 0)
    if step > 1:
        rows, cols = rows[::step], cols[::step]
    x = origin[0] + (cols.astype(np.float64) + 0.5) * RES
    y = origin[1] + ((h - 1 - rows).astype(np.float64) + 0.5) * RES
    return np.stack([x, y], 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', default='/tmp/g2p5fix_run')
    ap.add_argument('--tag', default='')
    ap.add_argument('--log', default='stdout.log')
    ap.add_argument('--render', default='')
    a = ap.parse_args()

    from scipy.spatial import cKDTree

    mapdir = os.path.join(a.run, 'data/new_map')
    img, w, h = read_pgm(os.path.join(mapdir, 'map.pgm'))
    P = read_pcd_xyz(os.path.join(mapdir, 'global.pcd')).astype(np.float64)
    pos, rpy, M = parse_log(os.path.join(a.run, a.log))

    tag = a.tag or os.path.basename(a.run.rstrip('/'))
    print('=' * 100)
    print('[%s] map.pgm %dx%d (%.1f x %.1f m) 占用格=%d 空闲格=%d 未知格=%d' %
          (tag, w, h, w * RES, h * RES, int((img == 0).sum()), int((img == 255).sum()), int((img == 128).sum())))

    # ---- 1) 栅格范围 vs 点云包围盒 ----
    import re
    my = open(os.path.join(mapdir, 'map.yaml')).read()
    ox = float(re.findall(r'origin:\s*\n\s*-\s*([-\d\.eE]+)', my)[0])
    oy = float(re.findall(r'origin:\s*\n\s*-\s*[-\d\.eE]+\s*\n\s*-\s*([-\d\.eE]+)', my)[0])
    gx = (ox, ox + w * RES)
    gy = (oy, oy + h * RES)
    px = (P[:, 0].min(), P[:, 0].max())
    py = (P[:, 1].min(), P[:, 1].max())
    print('  栅格范围 : x=[%.1f, %.1f] y=[%.1f, %.1f]   (origin=%.2f, %.2f)' % (gx[0], gx[1], gy[0], gy[1], ox, oy))
    print('  点云包围盒: x=[%.1f, %.1f] y=[%.1f, %.1f]' % (px[0], px[1], py[0], py[1]))
    print('  ★超出点云包围盒的余量: x %.1f/%.1f m, y %.1f/%.1f m  (正常应≈代码固定的 1 m 外扩)'
          % (px[0] - gx[0], gx[1] - px[1], py[0] - gy[0], gy[1] - py[1]))

    # ---- 2) 占用格 vs 点云 ----
    occ = occupied_world_xy(img, w, h, (ox, oy), step=1)
    tree = cKDTree(P[:, :2])
    d, _ = tree.query(occ, k=1, workers=-1)
    print('  占用格 -> 最近点云点(XY): 中位=%.3f m, p90=%.3f m, <0.3 m 占比=%.1f%%' %
          (np.median(d), np.percentile(d, 90), 100.0 * (d < 0.3).mean()))

    # ---- 3) 完备性：障碍点(离地 0.3~1.8 m) 是否被画成占用格 ----
    ttree = cKDTree(pos[:, :2])
    _, ti = ttree.query(P[:, :2], k=1, workers=-1)
    height_above_ground = P[:, 2] - (pos[ti, 2] - LIDAR_H)   # 用轨迹+实测安装高度估算"离地高度"
    obst = (height_above_ground > 0.3) & (height_above_ground < 1.8)
    ot = cKDTree(occ)
    do, _ = ot.query(P[obst][:, :2], k=1, workers=-1)
    print('  障碍点(离地0.3~1.8 m, %d 个) -> 最近占用格: 中位=%.3f m, <0.3 m 占比=%.1f%%'
          % (int(obst.sum()), np.median(do), 100.0 * (do < 0.3).mean()))

    if a.render:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 2, figsize=(16, 7))
        ax[0].imshow(img, cmap='gray', origin='upper',
                     extent=[gx[0], gx[1], gy[0], gy[1]])
        ax[0].plot(pos[:, 0], pos[:, 1], 'r-', lw=1.0, label='trajectory')
        ax[0].set_title('%s: map.pgm (0.05 m) + trajectory' % tag)
        ax[0].legend(loc='upper right')
        ax[0].set_aspect('equal')

        s = P[:: max(1, len(P) // 60000)]
        ax[1].scatter(s[:, 0], s[:, 1], s=0.2, c=s[:, 2], cmap='viridis')
        ax[1].plot(pos[:, 0], pos[:, 1], 'r-', lw=1.0)
        ax[1].set_title('%s: global.pcd top view (color = z) + trajectory' % tag)
        ax[1].set_aspect('equal')
        plt.tight_layout()
        plt.savefig(a.render, dpi=110)
        print('  已渲染: %s' % a.render)


if __name__ == '__main__':
    main()
