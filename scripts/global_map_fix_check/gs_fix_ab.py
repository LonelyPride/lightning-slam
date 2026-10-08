#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GS-FIX 补丁复核：用与"新保存地图"完全相同的 bag/日志/外参重建两版地图

  BUG = T_w_imu        * P_lidar   (= 打补丁前 GetGlobalMap 的做法)
  FIX = T_w_imu * T_imu_lidar * P_lidar (= 打补丁后的做法)

然后判定新保存的 global.pcd 究竟等于哪一版，并给出范围/平面度指标。

用法:
  python3 /tmp/gs_fix_ab.py [--log ..] [--yaml ..] [--bag ..] [--saved ..]
                            [--old-bug ..] [--offs 0,1,2,3]
"""
import argparse
import os
import sys

import numpy as np

import os  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_ab_map import (  # noqa: E402  复用已验证过的同一套实现
    Rz, Ry, Rx, parse_log, parse_extrinsic, SE3, load_raw_frames, voxel,
    nn_stats, planarity, read_pcd_xyz,
)


def describe(tag, P):
    m, p90 = planarity(P)
    print("  %-22s 点数=%8d  z=[%6.1f, %6.1f] 跨度=%5.1f m | 局部平面度RMS 中位=%.3f p90=%.3f m"
          % (tag, len(P), P[:, 2].min(), P[:, 2].max(), float(np.ptp(P[:, 2])), m, p90))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default='/tmp/gs_fix_run/stdout.log')
    ap.add_argument('--yaml', default='/tmp/gs_fix_run/airy96_final.yaml')
    ap.add_argument('--bag', default='/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07')
    ap.add_argument('--saved', default='/tmp/gs_fix_run/data/new_map/global.pcd')
    ap.add_argument('--old-bug', default='/tmp/planAplus_run/data/new_map/global.pcd')
    ap.add_argument('--topic', default='/lidar/front/points_raw')
    ap.add_argument('--offs', default='0,1,2,3')
    a = ap.parse_args()

    pos, rpy, M = parse_log(a.log)
    t_ext, R_ext = parse_extrinsic(a.yaml)
    T_imu_lidar = SE3(R_ext, t_ext)
    M4 = SE3(M, np.zeros(3))
    print("日志帧数 = %d" % len(pos))
    print("R_vehicle_imu(M) =\n%s" % np.round(M[:3, :3], 4))
    print("extrinsic(lidar->IMU) R =\n%s\n  t = %s" % (np.round(R_ext, 4), t_ext))

    frames = load_raw_frames(a.bag, a.topic, len(pos) + 8)
    print("bag 帧数 = %d" % len(frames))

    S = read_pcd_xyz(a.saved)
    print("\n[打补丁后新保存] %s" % a.saved)
    describe("新保存 global.pcd", S)

    old = None
    if a.old_bug and os.path.isfile(a.old_bug):
        old = read_pcd_xyz(a.old_bug)
        print("\n[打补丁前旧保存] %s" % a.old_bug)
        describe("旧(BUG) global.pcd", old)

    offs = [int(x) for x in a.offs.split(',') if x.strip() != '']
    verdict = {}
    for off in offs:
        P_bug, P_fix = [], []
        n = min(len(pos), len(frames) - off)
        for i in range(n):
            Tw_veh = SE3(Rz(rpy[i][2]) @ Ry(rpy[i][1]) @ Rx(rpy[i][0]), pos[i])
            Tw_imu = Tw_veh @ M4
            Q = voxel(frames[i + off])
            Qh = np.c_[Q, np.ones(len(Q))].T
            P_bug.append((Tw_imu @ Qh).T[:, :3])
            P_fix.append((Tw_imu @ T_imu_lidar @ Qh).T[:, :3])
        bug = np.vstack(P_bug).astype(np.float32)
        fix = np.vstack(P_fix).astype(np.float32)
        print("-" * 78)
        print("帧对齐偏移 off=%d => %d 帧, BUG 重建 %d 点, FIX 重建 %d 点" % (off, n, len(bug), len(fix)))
        for name, Mp in (('BUG', bug), ('FIX', fix)):
            mean, med, p95 = nn_stats(S, Mp)
            print("  新保存地图 -> %-3s 重建图: NN mean=%.3f median=%.3f p95=%.3f m" % (name, mean, med, p95))
            verdict.setdefault(off, {})[name] = med
        if old is not None:
            for name, Mp in (('BUG', bug), ('FIX', fix)):
                mean, med, p95 = nn_stats(old, Mp)
                print("  旧(BUG)地图 -> %-3s 重建图: NN mean=%.3f median=%.3f p95=%.3f m" % (name, mean, med, p95))
            mean, med, p95 = nn_stats(old, S)
            print("  旧(BUG)地图 -> 新保存地图: NN mean=%.3f median=%.3f p95=%.3f m (应≈%.1f，"
                  "证明两版确有本质差别)" % (mean, med, p95, 1.0))
        for name, Mp in (('BUG', bug), ('FIX', fix)):
            describe("%s 重建图" % name, Mp)

    print("=" * 78)
    for off in offs:
        b = verdict[off]['BUG']
        f = verdict[off]['FIX']
        ok = (f < 0.05) and (f < b / 5.0)
        print("off=%d: 新保存 vs FIX median=%.3f m, vs BUG median=%.3f m  => %s"
              % (off, f, b, "补丁生效（保存=正确的 FIX 版）" if ok else "异常：新保存不像 FIX 版，需复查"))
    print("判定阈值: 与 FIX 的 NN 中位 < 0.05 m 且 < 与 BUG 中位的 1/5")


if __name__ == '__main__':
    main()
