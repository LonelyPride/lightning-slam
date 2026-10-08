#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断：SaveMap 保存的点云地图 与 Pangolin 前端地图 的差别来源
Part1: 从 bag 直接读原始雷达帧（雷达系） -> 判断雷达方位覆盖 / 旋转对称性
Part2: 统计已保存的 global.pcd 结构
"""
import os, struct, sys
import numpy as np


def load_frames(bag_uri, topic, n=2):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while r.has_next() and len(out) < n:
        _, data, _ = r.read_next()
        out.append(deserialize_message(data, PointCloud2))
    return out


def msg_to_xyz(msg):
    dt_map = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    names, formats, offsets = [], [], []
    for fl in msg.fields:
        names.append(fl.name); formats.append(dt_map[fl.datatype]); offsets.append(fl.offset)
    dtype = np.dtype({'names': names, 'formats': formats,
                      'offsets': offsets, 'itemsize': msg.point_step})
    arr = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([arr['x'].astype(np.float64), arr['y'].astype(np.float64),
                    arr['z'].astype(np.float64)], 1)
    t = arr['timestamp'].astype(np.float64) if 'timestamp' in names else None
    return xyz, t, names


def nn_stats(A, B, sample=4000):
    from scipy.spatial import cKDTree
    idx = np.random.RandomState(0).choice(len(A), min(sample, len(A)), replace=False)
    d, _ = cKDTree(B).query(A[idx], k=1, workers=-1)
    return np.mean(d), np.median(d), np.percentile(d, 95)


def rot(axis, deg):
    a = np.deg2rad(deg); c, s = np.cos(a), np.sin(a)
    if axis == 'x': return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'y': return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def part1():
    bag = '/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07'
    topic = '/lidar/front/points_raw'
    print("=" * 78)
    print("[Part1] bag 原始雷达帧（雷达系）:", os.path.basename(bag))
    try:
        msgs = load_frames(bag, topic, 2)
    except Exception as e:
        print("  读取失败:", e); return
    print("  读到 %d 帧, frame_id=%s" % (len(msgs), msgs[0].header.frame_id))
    print("  fields = %s" % [(fl.name, fl.offset, fl.datatype) for fl in msgs[0].fields])
    for k, m in enumerate(msgs):
        xyz, ts, names = msg_to_xyz(m)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 0.1) & (r < 120)
        P = xyz[ok]
        print("-" * 70)
        print("  帧%d: width=%d height=%d point_step=%d 有效点=%d" % (k, m.width, m.height, m.point_step, len(P)))
        if ts is not None:
            print("       时间戳跨度 = %.4f s (%.1f Hz)" % (ts.max() - ts.min(), 1.0 / max(ts.max() - ts.min(), 1e-9)))
        az = np.rad2deg(np.arctan2(P[:, 1], P[:, 0])) % 360.0
        el = np.rad2deg(np.arcsin(np.clip(P[:, 2] / r[ok], -1, 1)))
        print("       距离: 中位=%.2f m, p5=%.2f p95=%.2f" % (np.median(r[ok]), np.percentile(r[ok], 5), np.percentile(r[ok], 95)))
        print("       仰角(相对雷达xy平面): p1=%.1f p50=%.1f p99=%.1f (度)" % tuple(np.percentile(el, [1, 50, 99])))
        h, e = np.histogram(az, bins=24, range=(0, 360))
        print("       方位角24桶直方图(0..360,步15°):")
        print("        ", " ".join("%5d" % v for v in h))
        print("        最小桶/最大桶 = %.2f  (接近1 => 每帧方位全覆盖 => 绕雷达z轴旋转对称)" % (h.min() / max(h.max(), 1)))
        for axis in ('x', 'y', 'z'):
            R = rot(axis, 90)
            mean, med, p95 = nn_stats(P, P @ R.T)
            print("       绕 %s 轴转 90°: 点到原点云 NN 距离 mean=%.4f median=%.4f p95=%.4f m" % (axis, mean, med, p95))
        break



def lzf_decompress(data, out_len):
    out = bytearray(out_len); i = 0; o = 0; n = len(data)
    while i < n:
        ctrl = data[i]; i += 1
        if ctrl < 32:
            ln = ctrl + 1; out[o:o+ln] = data[i:i+ln]; i += ln; o += ln
        else:
            ln = ctrl >> 5
            if ln == 7:
                ln += data[i]; i += 1
            ref = o - ((ctrl & 0x1f) << 8) - data[i] - 1; i += 1
            for _ in range(ln + 2):
                out[o] = out[ref]; o += 1; ref += 1
    return bytes(out)


def read_pcd_xyz(p):
    data = open(p, 'rb').read()
    hdr_end = data.find(b'DATA')
    hdr = data[:hdr_end].decode('ascii', 'ignore')
    npt = int([l for l in hdr.splitlines() if l.startswith('POINTS')][0].split()[1])
    fields = [l.split()[1:] for l in hdr.splitlines() if l.startswith('FIELDS')][0]
    sizes = [int(v) for v in [l for l in hdr.splitlines() if l.startswith('SIZE')][0].split()[1:]]
    if b'binary_compressed' in data[hdr_end:hdr_end + 40]:
        key = b'DATA binary_compressed\n'
        pos = data.find(key) + len(key)
        csize, usize = struct.unpack('<II', data[pos:pos + 8])
        raw = lzf_decompress(data[pos + 8:pos + 8 + csize], usize)
    else:
        key = b'DATA binary\n'
        pos = data.find(key) + len(key)
        raw = data[pos:]
    N = npt; off = 0; cols = {}
    for nm, sz in zip(fields, sizes):
        if nm in ('x', 'y', 'z'):
            cols[nm] = np.frombuffer(raw[off:off + N * 4], '<f4').astype(np.float64)
        off += N * sz
    return np.stack([cols['x'], cols['y'], cols['z']], 1)


def part2():
    print()
    print("=" * 78)
    print("[Part2] 已保存的 global.pcd 结构统计")
    from scipy.spatial import cKDTree
    cands = ['/tmp/planAplus_run/data/new_map/global.pcd',
             '/tmp/planA_run/data/new_map/global.pcd',
             '/tmp/lightning_run/data/new_map/global.pcd',
             '/opt/data/maps_3d/map01/global.pcd',
             '/home/h/map/global.pcd']
    for p in cands:
        if not os.path.exists(p):
            print("  (缺失) %s" % p); continue
        try:
            P = read_pcd_xyz(p)
        except Exception as e:
            print("  %s 读取失败 %s" % (p, e)); continue
        ok = np.isfinite(P).all(1)
        P = P[ok]
        print("-" * 70)
        print("  %s  (%.1f MB)" % (p, os.path.getsize(p) / 1e6))
        print("    点数 = %d" % len(P))
        for i, nm in enumerate('xyz'):
            print("      %s: [%.1f, %.1f] 跨度=%.1f" % (nm, P[:, i].min(), P[:, i].max(), P[:, i].ptp()))
        s = np.random.RandomState(0).choice(len(P), min(3000, len(P)), replace=False)
        d, _ = cKDTree(P).query(P[s], k=2, workers=-1)
        print("    局部最近邻间距: mean=%.4f median=%.4f p90=%.4f m" % (d[:, 1].mean(), np.median(d[:, 1]), np.percentile(d[:, 1], 90)))
        for ax, nm in ((2, 'z'), (0, 'x'), (1, 'y')):
            h, e = np.histogram(P[:, ax], bins=20)
            order = np.argsort(h)[::-1][:3]
            print("    %s 轴最密3片: %s (点数 %s)" % (nm, ["%.1f~%.1f" % (e[i], e[i + 1]) for i in order], [int(h[i]) for i in order]))


if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if which in ('all', '1'): part1()
    if which in ('all', '2'): part2()
