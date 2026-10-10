#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第6次修复（extrinsic_R=I + imu_acc_up_sign=-1）验收：用【工程自带脚本】的函数
   （scripts/global_map_fix_check/airy_ab_map.py: parse_log / read_pcd_xyz，
     逻辑同 scripts/global_map_fix_check/gs_ground_height.py）测两份地图：

     新 = 2026-10-09 12:09 离线重跑（/tmp/run6_offline.log + ws/data/new_map）
     旧 = 修复前 10:41 在线跑（已备份到 /tmp/dog_map_before_6th_fix_1207.tgz）

   输出：① R_vehicle_imu 的第 3 行（车体/世界"上"在 IMU 系的表达 —— 与 tf_static 链对比即可
         判定车体是否被装反了 180°）；② base_link rpy；③ 世界观下地面是否水平（法向 z 分量）
         与雷达/IMU 离地高度 h（应与 yaml 的 g2p5.lidar_height: 0.46 / floor_height: -0.46 相符）。

   备注：本脚本原为 /tmp/map6_check.py（2026-10-09 第 6 次修复的验收），原样归档；
         数据路径（两份地图 / 两份日志）写在文件末尾 __main__ 里，换数据改那里即可。
"""
import os
import re
import sys

import numpy as np

# 公共依赖在 ../global_map_fix_check/（用相对本文件的路径，整仓库搬走也能跑）
SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'global_map_fix_check')
sys.path.insert(0, SCRIPT_DIR)
from airy_ab_map import read_pcd_xyz  # noqa: E402

# 工程自带的 parse_log() 用 re.sub(r'[^\d\.\-\+\s]', ' ') 解析 R_vehicle_imu 的 3 行，
# 遇到科学计数（如 2.77556e-17 里的 'e'）会被砍成两个数 ⇒ 行列数不一致报错。
# 这里只在解析矩阵时换成"逐个抓浮点数"，位姿正则与工程脚本一致（都用 [-\\d\\.eE]+，本来就支持 e 记法）。
_FLOAT = re.compile(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?')
_POSE = re.compile(r'LIO state\(vehicle/base_link\):\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+),\s*'
                   r'rpy\(deg\):\s*\[\s*([-\d\.eE]+),\s*([-\d\.eE]+),\s*([-\d\.eE]+)\s*\]')
_ANSI = re.compile(r'\x1b\[[0-9;]*m')


def parse_log(log):
    """→ (poses[pos3], rpy3(rad), M=R_vehicle_imu)；与工程脚本 parse_log 等价但更稳。"""
    lines = [_ANSI.sub('', l) for l in open(log, errors='ignore').read().splitlines()]
    M = None
    for i, l in enumerate(lines):
        if 'R_vehicle_imu =' in l:
            rows = []
            for k in range(3):
                vals = [float(v) for v in _FLOAT.findall(lines[i + 1 + k])]
                rows.append(vals[:3])
            M = np.array(rows)
            break
    pos, rpy = [], []
    for l in lines:
        m = _POSE.search(l)
        if m:
            pos.append([float(m.group(1)), float(m.group(2)), float(m.group(3))])
            rpy.append(np.deg2rad([float(m.group(4)), float(m.group(5)), float(m.group(6))]))
    return np.array(pos), np.array(rpy), M

# tf_static 链（base_link→rslidartmp1 Ry110°→laser_points_front Rz-45°）给出的
#   车体"上"在雷达系(=本平台 IMU 系) 的表达，用于交叉验证 R_vehicle_imu 第 3 行
UP_FROM_TF = np.array([-0.6645, -0.6645, -0.3420])


def ang(u, v):
    # 先归一化：否则小角度会被浮点误差吃掉（0.49° 会因 dot>1 被 clip 显示成 0.00°）
    u = np.asarray(u, float) / np.linalg.norm(u)
    v = np.asarray(v, float) / np.linalg.norm(v)
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, float(np.dot(u, v)))))))


def check(tag, log, pcd):
    pos, rpy, M = parse_log(log)
    P = read_pcd_xyz(pcd).astype(np.float64)
    print('=' * 78)
    print('[%s]' % tag)
    print('  轨迹帧数=%d 地图点数=%d 世界系 z 范围=[%.2f, %.2f]'
          % (len(pos), len(P), P[:, 2].min(), P[:, 2].max()))

    if M is not None:
        up_row = M[2]
        print('  R_vehicle_imu 第3行(车体"上"在 IMU 系) = %s' % np.round(up_row, 4))
        print('    vs tf_static 链给出的车体"上" %s : 相差 %.2f deg  <-- 应 ≈0（修复前应 ≈180）'
              % (np.round(UP_FROM_TF, 4), ang(up_row, UP_FROM_TF)))

    if len(rpy):
        r, p, y = np.rad2deg(rpy[:, 0]), np.rad2deg(rpy[:, 1]), np.rad2deg(rpy[:, 2])
        print('  base_link rpy: roll 中位=%.3f deg [%.3f, %.3f] | pitch 中位=%.3f deg [%.3f, %.3f] | yaw 中位=%.2f deg'
              % (np.median(r), r.min(), r.max(), np.median(p), p.min(), p.max(), np.median(y)))

    hs, nzs, gaps = [], [], []
    for i in np.linspace(0, len(pos) - 1, 120).astype(int):
        p_ = pos[i]
        d = np.linalg.norm(P[:, :2] - p_[:2], axis=1)
        q = P[d < 8.0]
        if len(q) < 200:
            continue
        z = q[:, 2]
        g = q[z < np.percentile(z, 20)]          # 局部最低 20% ≈ 近似地面
        if len(g) < 50:
            continue
        c = g.mean(axis=0)
        A = g - c
        w, v = np.linalg.eigh(A.T @ A)
        n = v[:, 0]
        if n[2] < 0:
            n = -n
        nzs.append(n[2])
        hs.append(p_[2] - c[2])
        gaps.append(p_[2] - np.percentile(z, 5))
    hs, nzs = np.array(hs), np.array(nzs)
    print('  参与统计采样点=%d' % len(hs))
    print('  地面法向 z 分量: 中位=%.4f, 5%%~95%%=[%.3f, %.3f]  (≈1 ⇒ 世界系 z 向上、地面水平)'
          % (np.median(nzs), np.percentile(nzs, 5), np.percentile(nzs, 95)))
    print('  雷达/IMU 离地高度 h(轨迹z - 局部地面z): 中位=%.3f m, 5%%~95%%=[%.3f, %.3f] m'
          % (np.median(hs), np.percentile(hs, 5), np.percentile(hs, 95)))
    print('  冗余算法(轨迹z - 局部 z 的 5%% 分位): 中位=%.3f m' % np.median(gaps))
    print('  ★这条 h 是【整段轨迹】8 m 邻域"局部最低 20% 点"的统计，会被坡道/台阶/马路牙子污染，')
    print('    不是"传感器离地高度"；要报离地高度请用 frame_ground_check.py 的 B)（起点 3 m RANSAC = 0.503 m）。')


if __name__ == '__main__':
    check('新（第6次修复后，2026-10-09 12:09 离线重跑）',
          '/tmp/run6_offline.log',
          '/home/h/ddh/code/ws/lightning_ws/data/new_map/global.pcd')
    check('旧（修复前，10:41 在线跑，已备份解包）',
          '/tmp/run_slam_online.h-TX-Gaming-FX608JMR-FX608JMR.h.log.INFO.20261009-104117.18276',
          '/tmp/old_map_before_fix/data/new_map/global.pcd')
