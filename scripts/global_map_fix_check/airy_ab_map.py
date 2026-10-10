#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 bag 原始雷达帧 + 日志中的每帧位姿，重建两份地图：
   BUG  : 用 T_w_imu 直接变换【雷达系】点云 (= 当前 GetGlobalMap/SaveMap 的做法)
   FIX  : 用 T_w_imu * T_imu_lidar                          (= 老平台 lidar-slam 的做法)
   并比较：与已保存 global.pcd 的一致性、局部平面度、范围
"""
import os, re, struct, sys
import numpy as np

# 抓浮点数（含科学计数法），parse_log() 解析 R_vehicle_imu 三行时用
_FLOAT = re.compile(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?')


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def parse_log(log):
    """返回 (poses[pos3, rpy3(rad)] 列表, M=R_vehicle_imu)

    注意：R_vehicle_imu 的三行里可能出现科学计数（如 2.77556e-17）。
    早先这里用 re.sub(r'[^\\d\\.\\-\\+\\s]', ' ', ...) 清洗，会把 'e' 也当成杂字符删掉，
    于是 2.77556e-17 被切成两个数 ⇒ 行内个数不一致 ⇒ np.stack 直接报错。
    现在改成"逐个抓浮点数"（自然支持 e 记法），正常写法结果完全一致。
    """
    lines = open(log, errors='ignore').read().splitlines()
    M = None
    for i, l in enumerate(lines):
        if 'R_vehicle_imu =' in l:
            rows = []
            for k in range(3):
                vals = [float(v) for v in _FLOAT.findall(lines[i + 1 + k])]
                if len(vals) < 3:
                    raise ValueError('parse_log: R_vehicle_imu 第 %d 行不足 3 个数：%r'
                                     % (k + 1, lines[i + 1 + k]))
                rows.append(vals[:3])
            M = np.stack(rows)
            break
    pos, rpy = [], []
    pat = re.compile(r'LIO state\(vehicle/base_link\):\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+),\s*'
                     r'rpy\(deg\):\s*\[\s*([-\d\.eE]+),\s*([-\d\.eE]+),\s*([-\d\.eE]+)\s*\]')
    for l in lines:
        m = pat.search(l)
        if m:
            pos.append([float(m.group(1)), float(m.group(2)), float(m.group(3))])
            rpy.append(np.deg2rad([float(m.group(4)), float(m.group(5)), float(m.group(6))]))
    return np.array(pos), np.array(rpy), M


def parse_extrinsic(yaml_path):
    txt = open(yaml_path, errors='ignore').read()
    t = re.search(r'extrinsic_T:\s*\[([^\]]*)\]', txt)
    tv = [float(v) for v in t.group(1).replace('\n', ' ').split(',')]
    r = re.search(r'extrinsic_R:\s*\[([^\]]*)\]', txt)
    rv = [float(v) for v in r.group(1).replace('\n', ' ').split(',')]
    return np.array(tv), np.array(rv).reshape(3, 3)


def SE3(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def load_raw_frames(bag_uri, topic, n):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag_uri, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
    rd.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while rd.has_next() and len(out) < n:
        _, data, _ = rd.read_next()
        m = deserialize_message(data, PointCloud2)
        dt = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                       'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                       'offsets': [f.offset for f in m.fields], 'itemsize': m.point_step})
        a = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
        P = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float32)
        r = np.linalg.norm(P, axis=1)
        P = P[(r > 0.2) & (r < 80)]
        out.append(P)
    return out


def voxel(P, leaf=0.1):
    key = np.round(P / leaf).astype(np.int32)
    _, idx = np.unique(key, axis=0, return_index=True)
    return P[np.sort(idx)]



def nn_stats(A, B, sample=3000):
    from scipy.spatial import cKDTree
    idx = np.random.RandomState(1).choice(len(A), min(sample, len(A)), replace=False)
    d, _ = cKDTree(B).query(A[idx], k=1, workers=-1)
    return d.mean(), np.median(d), np.percentile(d, 95)


def planarity(P, k=20, sample=1500, seed=0):
    from scipy.spatial import cKDTree
    rs = np.random.RandomState(seed)
    idx = rs.choice(len(P), min(sample, len(P)), replace=False)
    nb = np.asarray(cKDTree(P).query(P[idx], k=k + 1, workers=-1)[1]).astype(np.int64)
    Q = P[nb[:, 1:]].astype(np.float64)   # (n, k, 3)
    Q = Q - Q.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', Q, Q) / k
    w = np.linalg.eigvalsh(cov)           # 升序
    sigma = np.sqrt(np.maximum(w[:, 0], 0))
    return np.median(sigma), np.percentile(sigma, 90)


def read_pcd_xyz(p):
    import os  # noqa: E402
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from airy_save_diag import read_pcd_xyz as r
    return r(p)


def main():
    log = '/tmp/planAplus_run/stdout4.log'
    yaml = '/tmp/planAplus_run/airy96_final.yaml'
    bag = '/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07'
    saved = '/tmp/planAplus_run/data/new_map/global.pcd'

    pos, rpy, M = parse_log(log)
    t_ext, R_ext = parse_extrinsic(yaml)
    T_imu_lidar = SE3(R_ext, t_ext)
    M = SE3(M, np.zeros(3))                       # R_vehicle_imu -> SE3
    print("日志帧数 = %d\nR_vehicle_imu(M) =\n%s" % (len(pos), np.round(M[:3, :3], 4)))
    print("extrinsic(lidar->IMU) R =\n%s\n t = %s" % (np.round(R_ext, 4), t_ext))

    frames = load_raw_frames(bag, "/lidar/front/points_raw", len(pos) + 8)
    print("bag 帧数 = %d" % len(frames))
    S = read_pcd_xyz(saved)
    print("已保存 global.pcd 点数 = %d" % len(S))
    m, p90 = planarity(S)
    print("已保存 global.pcd 自身: z=[%.1f, %.1f] 跨度=%.1f | 局部平面度RMS 中位=%.3f p90=%.3f m"
          % (S[:, 2].min(), S[:, 2].max(), S[:, 2].ptp(), m, p90))

    for off in (0, 1, 2, 3):
        P_bug, P_fix = [], []
        n = min(len(pos), len(frames) - off)
        for i in range(n):
            Tw_veh = SE3(Rz(rpy[i][2]) @ Ry(rpy[i][1]) @ Rx(rpy[i][0]), pos[i])
            Tw_imu = Tw_veh @ M
            Q = voxel(frames[i + off])
            Qh = np.c_[Q, np.ones(len(Q))].T
            P_bug.append((Tw_imu @ Qh).T[:, :3])
            P_fix.append((Tw_imu @ T_imu_lidar @ Qh).T[:, :3])
        bug = np.vstack(P_bug).astype(np.float32)
        fix = np.vstack(P_fix).astype(np.float32)
        print("-" * 74)
        print("帧对齐偏移 off=%d => %d 帧, BUG 图 %d 点, FIX 图 %d 点" % (off, n, len(bug), len(fix)))
        for name, Mp in (('BUG', bug), ('FIX', fix)):
            mean, med, p95 = nn_stats(S, Mp)
            print("   已保存地图 -> %-3s 重建图: NN mean=%.3f median=%.3f p95=%.3f m" % (name, mean, med, p95))
        for name, Mp in (('BUG', bug), ('FIX', fix)):
            z = Mp[:, 2]
            m, p90 = planarity(Mp)
            print("   %-3s 图: z=[%.1f, %.1f] 跨度=%.1f | 局部平面度RMS 中位=%.3f p90=%.3f m"
                  % (name, z.min(), z.max(), z.ptp(), m, p90))


if __name__ == '__main__':
    main()
