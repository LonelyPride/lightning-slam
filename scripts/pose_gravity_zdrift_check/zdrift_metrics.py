#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LIO 的 z 漂移指标（不需要地图，只用日志里的关键帧位姿）。

本 bag 是"下/上坡 + 原路返回"（全程 27 s，8.7 m），因此最硬的漂移指标是【回环误差】：
    车回到出发点时，LIO 给出的位置与出发点的差。真实漂移必须在这里体现，
    而"沿坡道上下"造成的 z 变化（本 bag ~1.3 m）是真实地形，不算漂移。

两个指标：
    1) |Δz| = |z_last - z_first|：回到起点后的竖直残差（对坡道 bag 也成立）
    2) |Δxy|：水平残差（水平方向没有坡道，最干净）

用法：
    python3 scripts/pose_gravity_zdrift_check/zdrift_metrics.py \
        --log /tmp/val_slam/run.log --tag '方案C+方向门限'
    # 另可加 --pcd data/new_map/0.pcd 做"局部地面高度/斜率"的辅助指标（依赖地图）

可选辅助指标（--pcd 给定时）：
    · 走廊内 传感器离地高度 h 的跨度（理想恒定）—— 注意：跨密度/跨地形不可直接比
    · 走廊地面平面拟合斜率（deg）+ rms —— 本 bag 车道含真实坡道，绝对值被地形污染
"""
import argparse
import math
import re
import struct
import sys

import numpy as np


def _lzf(data, expected):
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        ctrl = data[i]
        i += 1
        if ctrl < 32:
            ln = ctrl + 1
            out += data[i:i + ln]
            i += ln
        else:
            ln = ctrl >> 5
            ref = (ctrl & 0x1f) << 8
            if ln == 7:
                ln += data[i]
                i += 1
            ref += data[i]
            i += 1
            ln += 2
            start = len(out) - ref - 1
            for k in range(ln):
                out.append(out[start + k])
    return bytes(out[:expected])


def load_pcd(path):
    """只支持 PCD 的 DATA binary_compressed（本工程保存格式），返回 (N,3) float64。"""
    raw = open(path, 'rb').read()
    idx = raw.find(b'DATA binary_compressed')
    head = raw[:idx].decode(errors='ignore')
    hdr = {}
    for line in head.splitlines():
        p = line.split()
        if p:
            hdr[p[0]] = p[1:]
    npts = int(hdr['POINTS'][0])
    sizes = [int(x) for x in hdr['SIZE']]
    types = hdr['TYPE']
    counts = [int(x) for x in hdr['COUNT']]
    fields = hdr['FIELDS']
    body = raw[idx + len('DATA binary_compressed\n'):]
    clen, ulen = struct.unpack('<II', body[:8])
    dec = _lzf(body[8:8 + clen], ulen)
    tmap = {'F': 'f', 'U': 'I', 'I': 'i'}
    off = 0
    cols = {}
    for f, t, c, s in zip(fields, types, counts, sizes):
        cols[f] = np.frombuffer(dec, dtype=np.dtype('<' + tmap[t]), count=npts * c, offset=off).astype(np.float64)
        off += npts * c * s
    return np.stack([cols['x'], cols['y'], cols['z']], axis=1)


def load_kf(logp):
    """从日志（或 4 列 txt: id x y z）里取关键帧位置。"""
    kfs = []
    if logp.endswith('.txt'):
        for line in open(logp, errors='ignore'):
            p = line.split()
            if len(p) >= 4:
                kfs.append((int(p[0]), float(p[1]), float(p[2]), float(p[3])))
        return kfs
    pat = re.compile(r'LIO: create kf (\d+), state:\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)')
    for line in open(logp, errors='ignore'):
        m = pat.search(line)
        if m:
            kfs.append((int(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))))
    return kfs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True, help='run_slam_*.log.INFO.* 或 4 列 txt')
    ap.add_argument('--pcd', default=None, help='可选：data/<map>/0.pcd 或 global.pcd')
    ap.add_argument('--tag', default='')
    ap.add_argument('--kf-window', type=float, default=1.0, help='局部地面统计的窗口半径(m)')
    args = ap.parse_args()

    kfs = load_kf(args.log)
    if not kfs:
        print('日志里没有 "LIO: create kf" 行：%s' % args.log)
        return 1

    ids = [k[0] for k in kfs]
    P = np.array([[k[1], k[2], k[3]] for k in kfs])
    dz = abs(P[-1, 2] - P[0, 2])
    dxy = float(np.hypot(P[-1, 0] - P[0, 0], P[-1, 1] - P[0, 1]))
    print('=== %s' % (args.tag or args.log))
    print('    来源: %s' % args.log)
    print('    关键帧 %d 个 (id %d~%d)' % (len(kfs), min(ids), max(ids)))
    print('    ★回环残差: |Δz| = %.3f m   |Δxy| = %.3f m   （车回到出发点；真漂移看这里）' % (dz, dxy))
    print('      z: 首 %.3f 末 %.3f 最小 %.3f 最大 %.3f 跨度 %.3f m（含真实坡道升降，勿当漂移）'
          % (P[0, 2], P[-1, 2], P[:, 2].min(), P[:, 2].max(), P[:, 2].max() - P[:, 2].min()))
    print('      水平范围: x %.2f~%.2f  y %.2f~%.2f  |xy| 行程 %.2f m'
          % (P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max(),
             float(np.sum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)))))

    if args.pcd:
        M = load_pcd(args.pcd)
        print('    地图: %s  (%d 点)' % (args.pcd, len(M)))
        rows = []
        for i, x, y, z in kfs:
            m = (np.abs(M[:, 0] - x) < args.kf_window) & (np.abs(M[:, 1] - y) < args.kf_window)
            if m.sum() < 30:
                continue
            floor = float(np.percentile(M[m, 2], 5))
            rows.append((x, y, z, floor, z - floor))
        if len(rows) >= 3:
            A = np.array(rows)
            h = A[:, 4]
            print('    走廊内 传感器离地 h: 均值 %.3f 中位 %.3f 跨度 %.3f std %.3f (理想=恒定)'
                  % (h.mean(), float(np.median(h)), h.max() - h.min(), h.std()))
            X = np.c_[A[:, 0], A[:, 1], np.ones(len(A))]
            coef, _, _, _ = np.linalg.lstsq(X, A[:, 3], rcond=None)
            sl = math.hypot(coef[0], coef[1])
            print('    走廊地面平面拟合: 斜率 %.4f m/m = %.2f deg (方向 %.0f deg, rms %.3f m)'
                  % (sl, math.degrees(math.atan(sl)), math.degrees(math.atan2(coef[1], coef[0])),
                     float(np.sqrt(np.mean((X @ coef - A[:, 3]) ** 2)))))
            print('      ⚠ 本 bag 车道含真实坡道 ⇒ 该斜率绝对值是地形，不可当"世界系倾斜"判据；')
            print('        判定请用 lidar_ground_vs_gravity.py（近场地面法向 vs 加计重力）。')
        else:
            print('    走廊内关键帧太少(%d)，跳过局部地面统计' % len(rows))
    return 0


if __name__ == '__main__':
    sys.exit(main())

