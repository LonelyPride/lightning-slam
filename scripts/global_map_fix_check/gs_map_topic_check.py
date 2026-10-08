#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""栅格地图话题（默认 /localization/map，nav_msgs/OccupancyGrid）发布检查。

判据（两条互补，都不依赖 C++ 实现细节）：
  A) 频率与元数据：订阅 dur 秒，应收到 ≈ dur/interval 帧（默认 1 Hz）；
     info.width/height/resolution/origin/frame_id 应为合理的栅格地图参数。
  B) 内容一致性：与同目录【已保存】的栅格地图文件 map.pgm + map.yaml 逐格比对
     （默认 /home/h/ddh/code/ws/lightning_ws/data/new_map）：
       · 元数据（width/height/resolution/origin）必须一致（map.yaml 的 origin 即 data[0] 的世界坐标）；
       · 同一世界坐标下的格子取值应当一致（100=占据 / 0=空闲 / -1=未知）；
       · 单独统计"占据↔空闲"互翻的格子数 —— 这类错误对下游规划最危险（会凭空多/少障碍物）。
     行序约定（与 slam.cc:SaveMap 写图、gs_g2p5_check.py:occupied_world_xy 一致）：
     图像第 0 行是地图【顶部】(y 最大)，OccupancyGrid 的 data 从【左下角】开始按行排。

用法:
  # 最小自检：只看有没有在发、频率对不对（run_loc_online 跑起来后）
  python3 gs_map_topic_check.py --dur 6

  # 完整比对：额外跟已保存的地图文件逐格比
  python3 gs_map_topic_check.py --dur 6 --ref /home/h/ddh/code/ws/lightning_ws/data/new_map

  # 再加行序/朝向的独立判据：用日志里的车辆轨迹（车走过的路必须是空闲格）
  #   --ref-log 可以是任一含车辆轨迹的 INFO 日志（建图日志 / 定位日志都行）
  python3 gs_map_topic_check.py --dur 6 --ref data/new_map --ref-log /tmp/run_loc_online.INFO

  # 换话题/换间隔（与 config 的 system.publish_map.* 对应）
  python3 gs_map_topic_check.py --topic /localization/map --interval 1.0

退出码：0 = 通过；1 = 未收到消息/频率或内容不符（便于挂在检查脚本里）。
"""
import argparse
import os
import re
import sys
import time

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


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


def read_map_yaml(path):
    """读 slam.cc:SaveMap 写出的 map.yaml -> (分辨率, origin_x, origin_y, 占用阈值, 空闲阈值, negate)"""
    if yaml is None:
        raise RuntimeError('需要 python3-yaml：pip3 install pyyaml')
    with open(path) as f:
        meta = yaml.safe_load(f)
    origin = list(meta.get('origin', [0.0, 0.0, 0.0]))
    return (float(meta.get('resolution', 0.05)), float(origin[0]), float(origin[1]),
            float(meta.get('occupied_thresh', 0.65)), float(meta.get('free_thresh', 0.25)),
            int(meta.get('negate', 0)))


def pgm_to_occ_grid(img, occ_th, free_th, negate):
    """map.pgm -> 与 OccupancyGrid.data 同约定的二维数组 data[y, x]（y 从 origin 起向上）"""
    prob = img.astype(np.float64) / 255.0
    occ = prob if negate else (1.0 - prob)
    out = np.full(img.shape, -1, dtype=np.int8)
    out[occ > occ_th] = 100
    out[occ < free_th] = 0
    return out[::-1, :]          # 图像第 0 行是地图顶部 ⇒ 翻转成 data 的行序（y 向上）




def stat(arr):
    return int((arr == 100).sum()), int((arr == 0).sum()), int((arr == -1).sum())


def traj_free_ratio(grid, reso, ox, oy, log, tag):
    """行序/朝向的【独立】判据：建图日志里的车辆轨迹必须落在"空闲"格上。

    轨迹取 laser_mapping.cc 打印的 `LIO state(vehicle/base_link): x y z`（与 g2p5 栅格图同世界系）。
    正确行序 ⇒ 轨迹几乎全在 free(0)；上下翻转 ⇒ 会落到 unknown(-1) 区域（实测 100% 未知）。
    该判据不看 map.pgm 的像素分布，只依赖\"车走过的路一定是空闲\"这一物理事实。
    """
    pat = re.compile(r'LIO state\(vehicle/base_link\):\s*([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)')
    pos = np.array([[float(a), float(b), float(c)]
                    for a, b, c in pat.findall(open(log, errors='ignore').read())])
    if len(pos) < 10:
        print('  ⚠ %s 里没找到足够的 LIO 轨迹行，跳过行序判据' % log)
        return None, None
    h, w = grid.shape
    ix = np.floor((pos[:, 0] - ox) / reso).astype(int)
    iy = np.floor((pos[:, 1] - oy) / reso).astype(int)
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    f_ok = 100.0 * (grid[iy[inside], ix[inside]] == 0).mean()
    f_bad = 100.0 * ((grid[::-1, :])[iy[inside], ix[inside]] == 0).mean()
    print('  [%s] 轨迹点数=%d（%.1f%% 落在地图内, x=[%.1f,%.1f] y=[%.1f,%.1f]）'
          % (tag, len(pos), 100.0 * inside.mean(), pos[:, 0].min(), pos[:, 0].max(),
             pos[:, 1].min(), pos[:, 1].max()))
    print('  ★ 行序判据：正确行序下轨迹落在空闲格 = %.1f%%，上下翻转后 = %.1f%%（后者应明显更低）'
          % (f_ok, f_bad))
    return f_ok, f_bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/localization/map', help='要订阅的话题')
    ap.add_argument('--dur', type=float, default=6.0, help='订阅时长(秒)')
    ap.add_argument('--interval', type=float, default=1.0, help='配置的发布间隔(秒)，用于频率判据')
    ap.add_argument('--ref', default='', help='已保存地图目录(含 map.pgm/map.yaml)，留空则跳过内容比对')
    ap.add_argument('--ref-log', default='',
                    help='建图日志(含 LIO state(vehicle/base_link) 行)：用车辆轨迹落格独立判定行序/朝向')
    ap.add_argument('--min-rate-ratio', type=float, default=0.7, help='实测频率/期望频率 的下限')
    ap.add_argument('--min-agree', type=float, default=0.99, help='与参考地图同格一致率下限')
    ap.add_argument('--max-flip-ratio', type=float, default=0.001,
                    help='占据↔空闲互翻比例上限：发"已保存的地图文件"时应恰为 0')
    ap.add_argument('--dump', default='', help='把最后一帧存成 npz（便于后续离线比对）')
    a = ap.parse_args()

    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    import rclpy
    from nav_msgs.msg import OccupancyGrid

    rclpy.init()
    node = rclpy.create_node('gs_map_topic_check')

    received = []
    state = {'msg': None}

    def cb(msg):
        received.append(time.time())
        state['msg'] = msg

    # 发布端是 rclcpp::QoS(1)（reliable / volatile），这里用兼容的 reliable+volatile
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(OccupancyGrid, a.topic, cb, qos)

    t_start = time.time()
    while time.time() - t_start < a.dur:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    print('=' * 100)
    print('[topic] %s  订阅 %.1f s，收到 %d 帧' % (a.topic, a.dur, len(received)))
    if not received:
        print('★ 未收到任何消息：检查 run_loc_online 是否在跑、system.publish_map.enable 是否为 true、'
              '以及地图目录（system.publish_map.path / system.map_path）下的 map.pgm 是否存在')
        return 1

    span = received[-1] - received[0]
    rate = (len(received) - 1) / span if span > 1e-6 else float('inf')
    print('  首帧延迟=%.2f s（订阅时刻 → 第一帧）' % (received[0] - t_start))
    if len(received) > 1:
        gaps = np.diff(received)
        print('  实测频率=%.3f Hz（期望 %.3f Hz；间隔 中位=%.3f s, min=%.3f, max=%.3f）'
              % (rate, 1.0 / a.interval, float(np.median(gaps)), float(gaps.min()), float(gaps.max())))
    else:
        print('  ★ 只收到 1 帧，无法统计频率')

    msg = state['msg']
    w, h = int(msg.info.width), int(msg.info.height)
    reso = float(msg.info.resolution)
    ox, oy = float(msg.info.origin.position.x), float(msg.info.origin.position.y)
    data = np.array(msg.data, dtype=np.int8).reshape(h, w)
    o, f, u = stat(data)
    print('  末帧: %dx%d @ %.3f m, origin=(%.2f, %.2f), frame_id="%s"' % (w, h, reso, ox, oy, msg.header.frame_id))
    print('        世界范围 x=[%.2f, %.2f] y=[%.2f, %.2f]；占据=%d 空闲=%d 未知=%d（%.1f%% 未知）'
          % (ox, ox + w * reso, oy, oy + h * reso, o, f, u, 100.0 * u / (w * h)))

    fail = []
    if len(received) > 1 and rate < a.min_rate_ratio / a.interval:
        fail.append('频率过低(%.3f Hz < %.3f Hz)' % (rate, a.min_rate_ratio / a.interval))
    if not msg.header.frame_id:
        fail.append('header.frame_id 为空')
    if o + f == 0:
        fail.append('格子全是未知(-1)，等于没有地图内容')

    # ---- 与已保存的栅格地图文件逐格比对 ----
    if a.ref:
        pgm_path = os.path.join(a.ref, 'map.pgm')
        yml_path = os.path.join(a.ref, 'map.yaml')
        if not (os.path.exists(pgm_path) and os.path.exists(yml_path)):
            fail.append('参考地图文件不存在: %s 或 %s' % (pgm_path, yml_path))
        else:
            img, rw, rh = read_pgm(pgm_path)
            r_reso, r_ox, r_oy, occ_th, free_th, negate = read_map_yaml(yml_path)
            ref = pgm_to_occ_grid(img, occ_th, free_th, negate)
            ro, rf, ru = stat(ref)
            print('  参考 map.pgm: %dx%d @ %.3f m, origin=(%.2f, %.2f)；占据=%d 空闲=%d 未知=%d'
                  % (rw, rh, r_reso, r_ox, r_oy, ro, rf, ru))

            if abs(reso - r_reso) > 1e-6:
                fail.append('分辨率不一致: 话题 %.6f vs map.yaml %.6f' % (reso, r_reso))
            else:
                # 只比世界坐标下的交集：允许两者尺寸/origin 不同（live 源的地图还在长大）
                x0, x1 = max(ox, r_ox), min(ox + w * reso, r_ox + rw * r_reso)
                y0, y1 = max(oy, r_oy), min(oy + h * reso, r_oy + rh * r_reso)
                if x1 <= x0 or y1 <= y0:
                    fail.append('话题地图与参考地图没有重叠区域')
                else:
                    i0x, i1x = int(round((x0 - ox) / reso)), int(round((x1 - ox) / reso))
                    i0y, i1y = int(round((y0 - oy) / reso)), int(round((y1 - oy) / reso))
                    j0x, j1x = int(round((x0 - r_ox) / r_reso)), int(round((x1 - r_ox) / r_reso))
                    j0y, j1y = int(round((y0 - r_oy) / r_reso)), int(round((y1 - r_oy) / r_reso))
                    A = data[i0y:i1y, i0x:i1x]
                    B = ref[j0y:j1y, j0x:j1x]
                    agree = float((A == B).mean()) if A.size else 0.0
                    flip = int((((A == 100) & (B == 0)) | ((A == 0) & (B == 100))).sum())
                    flip_ratio = flip / float(A.size) if A.size else 0.0
                    print('  ★ 重叠区 %d 格（%.1f x %.1f m）：同格一致率=%.4f，占据↔空闲互翻=%d 格（%.4f%%）'
                          % (A.size, (x1 - x0), (y1 - y0), agree, flip, 100.0 * flip_ratio))
                    if agree < a.min_agree:
                        fail.append('与已保存地图的一致率 %.4f < %.4f' % (agree, a.min_agree))
                    if flip_ratio > a.max_flip_ratio:
                        fail.append('"占据↔空闲"互翻 %d 格（%.4f%%）超过阈值 %.4f%% ⇒ 行序/翻转或阈值处理有误'
                                    % (flip, 100.0 * flip_ratio, 100.0 * a.max_flip_ratio))
                    elif flip:
                        print('  （互翻占比未超阈值：本功能发的是\"已保存的地图文件\"，正常应恰为 0 格；'
                              '若非 0 说明比对期间地图被重新保存过）')
                    if (w, h) == (rw, rh) and abs(ox - r_ox) < 1e-6 and abs(oy - r_oy) < 1e-6:
                        print('  （尺寸/origin 与 map.pgm 完全相同 ⇒ 属于\"直接发已保存地图\"）')

    # ---- 行序/朝向的独立判据：用建图日志里的车辆轨迹（与栅格图同世界系）----
    if a.ref_log:
        f_ok, f_bad = traj_free_ratio(data, reso, ox, oy, a.ref_log, 'published')
        if f_ok is not None and (f_ok < f_bad or f_ok < 90.0):
            fail.append('行序/朝向判据不通过：轨迹落在空闲格仅 %.1f%%（翻转后 %.1f%%）' % (f_ok, f_bad))

    if a.dump:
        np.savez_compressed(a.dump, data=data, width=w, height=h, resolution=reso,
                            origin=np.array([ox, oy]), frame_id=msg.header.frame_id)
        print('  已导出: %s' % a.dump)
    print('-' * 100)
    if fail:
        for x in fail:
            print('★ FAIL: %s' % x)
        return 1
    print('★ PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
