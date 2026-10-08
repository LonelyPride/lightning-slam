#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位端验证：把 lidar_loc 输出的位姿与"同一条 bag 的 SLAM/LIO 轨迹"比对。

由于定位用的地图就是同一条 bag 建出来的（世界系完全相同，两次运行的首个关键帧状态逐位一致），
所以可以直接把定位位姿与 LIO 轨迹做最近邻比较来衡量定位精度/是否跑偏。

用法:
  python3 gs_loc_check.py --loc-log /tmp/loc_fixrun/stdout.log --ref-log /tmp/g2p5fix_run/stdout.log [--tag L1]
"""
import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_ab_map import parse_log  # noqa: E402


def parse_loc_poses(path):
    """取 lidar_loc.cc:890 那一行的 'confidence: X, t: x y z' —— 即每帧激光定位输出（T_map_lidar）"""
    pat = re.compile(r'confidence:\s*([-\d\.eE]+),\s*t:\s*([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+),\s*succ:\s*(\d)')
    conf, pos = [], []
    for l in open(path, errors='ignore'):
        m = pat.search(l)
        if m:
            conf.append(float(m.group(1)))
            pos.append([float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(conf), np.array(pos)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--loc-log', default='/tmp/loc_fixrun/stdout.log')
    ap.add_argument('--ref-log', default='/tmp/g2p5fix_run/stdout.log')
    ap.add_argument('--tag', default='')
    a = ap.parse_args()

    from scipy.spatial import cKDTree

    conf, lp = parse_loc_poses(a.loc_log)
    rpos, rpy, M = parse_log(a.ref_log)
    tag = a.tag or os.path.basename(os.path.dirname(a.loc_log))
    print('=' * 96)
    print('[%s] 定位输出帧数=%d, 参考(LIO)轨迹帧数=%d' % (tag, len(conf), len(rpos)))
    ok = conf > 1.8
    print('  匹配分值(阈值 min_init_confidence=1.8): 中位=%.3f, min=%.3f, max=%.3f；>1.8 的比例=%.1f%%'
          % (np.median(conf), conf.min(), conf.max(), 100.0 * ok.mean()))

    tree = cKDTree(rpos)
    d, _ = tree.query(lp, k=1, workers=-1)
    d3 = np.linalg.norm(lp - rpos[tree.query(lp, k=1, workers=-1)[1]], axis=1)
    print('  定位位姿 -> 最近 LIO 轨迹点: XY 中位=%.3f m, p90=%.3f m, max=%.3f m' %
          (np.median(d), np.percentile(d, 90), d.max()))
    print('                                3D 中位=%.3f m, max=%.3f m' % (np.median(d3), d3.max()))
    print('  定位轨迹范围: x=[%.1f, %.1f] y=[%.1f, %.1f]   LIO 轨迹范围: x=[%.1f, %.1f] y=[%.1f, %.1f]'
          % (lp[:, 0].min(), lp[:, 0].max(), lp[:, 1].min(), lp[:, 1].max(),
             rpos[:, 0].min(), rpos[:, 0].max(), rpos[:, 1].min(), rpos[:, 1].max()))


if __name__ == '__main__':
    main()
