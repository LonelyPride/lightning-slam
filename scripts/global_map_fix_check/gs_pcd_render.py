#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补丁前后两张已保存 global.pcd 的同视角渲染对比（俯视 + 侧视），作为目视证据。
   左列 = 打补丁后新保存（正确拼接），右列 = 打补丁前旧保存（漏外参）。
   俯视图按高度着色：正确时地面是一片连续的单层平面；出错时同一片地面被摊成
   多层不同高度的"壳"，环视一圈会看到同心/条带的伪影。
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

import os  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_save_diag import read_pcd_xyz

NEW = '/tmp/gs_fix_run/data/new_map/global.pcd'
OLD = '/tmp/planAplus_run/data/new_map/global.pcd'
OUT = '/tmp/gs_pcd_compare.png'


def stats(P, tag):
    print("%-14s 点数=%7d  x=[%7.1f,%7.1f] y=[%7.1f,%7.1f] z=[%7.1f,%7.1f]  z跨度=%5.1f m"
          % (tag, len(P), P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max(),
             P[:, 2].min(), P[:, 2].max(), P[:, 2].ptp()))


def main():
    new = read_pcd_xyz(NEW).astype(np.float64)
    old = read_pcd_xyz(OLD).astype(np.float64)
    stats(new, "新保存(补丁后)")
    stats(old, "旧保存(补丁前)")

    vmin = min(new[:, 2].min(), old[:, 2].min())
    vmax = max(np.percentile(new[:, 2], 99.5), np.percentile(old[:, 2], 99.5))

    fig, axes = plt.subplots(2, 2, figsize=(22, 13), facecolor='white')
    for col, (P, name) in enumerate([(new, "打补丁后（正确：含 雷达->IMU 外参）"),
                                     (old, "打补丁前（BUG：漏 雷达->IMU 外参）")]):
        ax = axes[0][col]
        ax.scatter(P[:, 0], P[:, 1], c=P[:, 2], s=0.7, cmap='jet', vmin=vmin, vmax=vmax, linewidths=0)
        ax.set_title("%s\n俯视 XY（按高度着色）" % name, fontsize=15)
        ax.set_xlabel("x / m"); ax.set_ylabel("y / m")
        ax.set_aspect('equal'); ax.grid(alpha=0.25)

        ax = axes[1][col]
        ax.scatter(P[:, 0], P[:, 2], c=P[:, 2], s=0.7, cmap='jet', vmin=vmin, vmax=vmax, linewidths=0)
        ax.set_title("%s\n侧视 XZ（x 沿轨迹 / z 为高度）" % name, fontsize=15)
        ax.set_xlabel("x / m"); ax.set_ylabel("z / m")
        ax.axhline(0.0, color='k', lw=0.8, ls='--', alpha=0.6)
        ax.grid(alpha=0.25)
    fig.suptitle("global.pcd 同 bag 重跑对比：补丁生效（地面由多层壳 -> 单层平面，z 跨度 %.1f m -> %.1f m）"
                 % (old[:, 2].ptp(), new[:, 2].ptp()), fontsize=17)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(OUT, dpi=110)
    print("已保存对比图:", OUT)


if __name__ == '__main__':
    main()
