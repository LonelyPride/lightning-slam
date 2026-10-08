#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位端"相对运动递推(先验)"坐标系修复的量级评估（对应 Localization::ToLidarFrameState）。

问题：LIO 给出的 LO/DR 是【IMU 系】位姿，而 lidar_loc/PGO 内部是用它和【雷达系】绝对位姿做
      相对运动递推：
          补丁前: guess = T_map_lidar(t1) * (T_imu(t1)_imu(t2))            ← 把 IMU 系增量当雷达系用
          补丁后: 入口处先把 LO/DR 换成雷达系，即等价于
                  guess = T_map_lidar(t1) * (T_imu_lidar⁻¹ * Δ_imu * T_imu_lidar)
      两者之差 = 把"IMU 系增量"用 90° 外参共轭了一下，横向/垂向会互换。

本脚本用 bag 的真实位姿序列给出该差值的实测量级（平移/姿态）。
用法: python3 gs_loc_prior_check.py [--log ..] [--yaml ..]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airy_ab_map import Rx, Ry, Rz, SE3, parse_extrinsic, parse_log  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default='/tmp/g2p5fix_run/stdout.log')
    ap.add_argument('--yaml', default='/tmp/g2p5fix_run/airy96_final.yaml')
    a = ap.parse_args()

    pos, rpy, M = parse_log(a.log)
    t_ext, R_ext = parse_extrinsic(a.yaml)
    T_imu_lidar = SE3(R_ext, t_ext)
    T_lidar_imu = np.linalg.inv(T_imu_lidar)
    M4 = SE3(M, np.zeros(3))

    T_w_lidar = []
    for i in range(len(pos)):
        T_w_imu = SE3(Rz(rpy[i][2]) @ Ry(rpy[i][1]) @ Rx(rpy[i][0]), pos[i]) @ M4
        T_w_lidar.append(T_w_imu @ T_imu_lidar)

    dt_err, dr_err, dt_motion, steps = [], [], [], []
    for i in range(1, len(T_w_lidar)):
        # 该帧的 IMU 系绝对位姿（用于构造"IMU 系相对增量"）
        T_w_imu_prev = np.linalg.inv(T_imu_lidar) @ T_w_lidar[i - 1]
        T_w_imu_cur = np.linalg.inv(T_imu_lidar) @ T_w_lidar[i]
        d_imu = np.linalg.inv(T_w_imu_prev) @ T_w_imu_cur   # Δ in IMU frame

        pred_code = T_w_lidar[i - 1] @ d_imu                       # 补丁前：直接乘
        pred_fix = T_w_lidar[i - 1] @ (T_lidar_imu @ d_imu @ T_imu_lidar)  # 补丁后

        dt_err.append(np.linalg.norm(pred_code[:3, 3] - pred_fix[:3, 3]))
        Rr = pred_code[:3, :3].T @ pred_fix[:3, :3]
        dr_err.append(np.degrees(np.arccos(np.clip((np.trace(Rr) - 1) / 2, -1, 1))))
        dt_motion.append(np.linalg.norm(d_imu[:3, 3]))
        steps.append(np.linalg.norm(pos[i] - pos[i - 1]))

    dt_err = np.array(dt_err)
    dr_err = np.array(dr_err)
    dt_motion = np.array(dt_motion)
    print('帧数=%d, 相邻帧平均位移=%.4f m, 平均 IMU 系增量平移=%.4f m' %
          (len(dt_err), np.mean(steps), np.mean(dt_motion)))
    print('补丁前/后 的先验位姿差: 平移 中位=%.4f m, p95=%.4f m, max=%.4f m' %
          (np.median(dt_err), np.percentile(dt_err, 95), dt_err.max()))
    print('                        姿态 中位=%.3f deg, max=%.3f deg' % (np.median(dr_err), dr_err.max()))
    print('相对本帧真实位移的比例: 平移差 中位=%.1f%% (横向分量会被 90° 外参换到垂向)' %
          (100.0 * np.median(dt_err) / max(np.mean(steps), 1e-9)))


if __name__ == '__main__':
    main()
