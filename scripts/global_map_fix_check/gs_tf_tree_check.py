#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位端 TF 树检查（默认期望：map → odom → base_footprint → base_link → ...）。

背景：车上的 TF 树本来就有 odom→base_footprint（动态）与 base_footprint→base_link（静态），
      而本工程原来【直接发 map→base_link】，于是 base_link 出现两个父节点 ⇒ TF 树冲突。
      现在定位只补 map→odom 这一条边：T_map_odom = T_map_base · (T_odom_base)⁻¹。

判据（都不依赖 C++ 实现细节）：
  A) 单一父节点：订阅 /tf 与 /tf_static dur 秒，统计每个 child frame 的父帧集合。
     mode=map_odom（默认）时期望 base_link 的父帧【只有一个】且不是 map ——
     若看到 {base_footprint, map} 就是"定位还在直接发 map→base_link"的冲突状态。
  B) 定位补的那条边存在：tf2 能解出 map→odom，且自检
        T_map_base(解出) ≟ T_map_odom · T_odom_base   （误差 < 1e-6）
     成立说明发布出来的是严格的坐标系合成，而不是随手拼的数值。
  C) 树是连通的：用 tf2 解析 map→base_link（RViz / Nav2 的查法）必须成功，并打印该位姿。

用法:
  # 定位跑起来之后（车上 TF 与定位端都在发 TF）
  python3 gs_tf_tree_check.py --dur 8

  # 自定义帧名
  python3 gs_tf_tree_check.py --dur 8 --map-frame map --odom-frame odom --base-frame base_link

  # 检查老行为（config 里 system.tf.mode = map_base）：此时允许 map 直接作为 base_link 的父帧
  python3 gs_tf_tree_check.py --dur 8 --mode map_base

退出码：0 = 通过；1 = 未通过（便于挂在检查脚本里）。
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


# --- 四元数/位姿小工具（纯 python，避免额外依赖）--------------------------------


def quat_to_mat(x, y, z, w):
    """四元数 -> 3x3 旋转矩阵（行主序 list[list]）"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def tf_to_se3(tf_msg):
    """TransformStamped -> (R(3x3 list), t(list[3]))"""
    tr = tf_msg.transform.translation
    ro = tf_msg.transform.rotation
    return quat_to_mat(ro.x, ro.y, ro.z, ro.w), [tr.x, tr.y, tr.z]


def compose(a, b):
    """(Ra, ta) ∘ (Rb, tb)"""
    ra, ta = a
    rb, tb = b
    r = [[sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    t = [sum(ra[i][k] * tb[k] for k in range(3)) + ta[i] for i in range(3)]
    return r, t


def se3_diff(a, b):
    """两个 (R,t) 的最大绝对差"""
    return max(max(abs(a[0][i][j] - b[0][i][j]) for j in range(3)) for i in range(3)) + \
        max(abs(a[1][i] - b[1][i]) for i in range(3))


def yaw_of(r):
    return math.degrees(math.atan2(r[1][0], r[0][0]))


# --- 探针 ----------------------------------------------------------------------


class TfProbe(Node):
    """统计每个 child frame 的父帧集合，并提供 tf2 查询能力"""

    def __init__(self):
        super().__init__('gs_tf_tree_probe')
        self.parents = {}  # child -> set(parent)
        self.count = 0
        self.max_stamp = 0.0  # 收到的最新 TF 时间戳（用于"同一时刻"查询）

        dynamic_qos = QoSProfile(depth=500, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        # /tf_static 必须用 transient_local，否则晚加入的订阅者收不到那些只发一次的消息
        static_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(TFMessage, '/tf', self.on_tf, dynamic_qos)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf, static_qos)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def on_tf(self, msg):
        for t in msg.transforms:
            self.parents.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
            self.count += 1
            stamp = t.header.stamp.sec + 1e-9 * t.header.stamp.nanosec
            if stamp > self.max_stamp:
                self.max_stamp = stamp

    def lookup(self, target, source, stamp=None):
        """查询（stamp=None 表示取最新可用）。失败返回 None（tf2 的异常类型较多，这里统一兜住）"""
        if stamp is None:
            when = Time()
        else:
            when = Time(seconds=int(stamp), nanoseconds=int(round((stamp - int(stamp)) * 1e9)))
        try:
            return self.buffer.lookup_transform(target, source, when)
        except Exception:  # noqa: BLE001
            return None


def main():
    ap = argparse.ArgumentParser(description='定位端 TF 树检查（map→odom→…→base_link）')
    ap.add_argument('--dur', type=float, default=8.0, help='订阅/采样时长(秒)，默认 8')
    ap.add_argument('--map-frame', default='map', help='地图帧名，默认 map')
    ap.add_argument('--odom-frame', default='odom', help='里程计帧名，默认 odom')
    ap.add_argument('--base-frame', default='base_link', help='车体帧名，默认 base_link')
    ap.add_argument('--mode', default='map_odom', choices=['map_odom', 'map_base'],
                    help='map_odom(默认)=定位发 map→odom；map_base=定位直接发 map→base_link（老行为）')
    args = ap.parse_args()

    rclpy.init()
    node = TfProbe()

    t0 = time.time()
    while time.time() - t0 < args.dur:
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = True
    print('=== 采样 %.1f s：收到 %d 条 TF，涉及 %d 个 child frame ===' % (args.dur, node.count, len(node.parents)))
    if node.count == 0:
        print('✗ 没收到任何 TF：车上的里程计/底盘节点是否在发？ ros2 topic hz /tf')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print('--- 父帧一览（child <- parents）---')
    for child in sorted(node.parents):
        print('  %-22s <- %s' % (child, ', '.join(sorted(node.parents[child]))))

    # --- A) 单一父节点 ---------------------------------------------------------
    base_parents = node.parents.get(args.base_frame, set())
    print('--- A) %s 的父帧: %s ---' % (args.base_frame, sorted(base_parents) if base_parents else '（未收到）'))
    if args.mode == 'map_odom':
        if not base_parents:
            print('  ✗ 没收到 %s 的 TF（车上 TF 树不完整？）' % args.base_frame)
            ok = False
        elif len(base_parents) == 1 and args.map_frame not in base_parents:
            print('  ✓ %s 只有一个父帧 —— 定位没有直接发 %s→%s，不再有双父节点'
                  % (args.base_frame, args.map_frame, args.base_frame))
        elif args.map_frame in base_parents:
            print('  ✗ %s 的父帧里出现了 %s ⇒ TF 树冲突（定位仍在直接发 %s→%s；'
                  '请确认 config 的 system.tf.mode = map_odom）'
                  % (args.base_frame, args.map_frame, args.map_frame, args.base_frame))
            ok = False
        else:
            print('  ✗ %s 有多个父帧 %s ⇒ TF 树冲突' % (args.base_frame, sorted(base_parents)))
            ok = False
    else:
        print('  （mode=map_base：允许 %s 直接作为 %s 的父帧，本项不判）' % (args.map_frame, args.base_frame))

    # --- B) map→odom 存在 + 合成自检 -------------------------------------------
    # 用【同一时刻】查询三条边：时间不同步（各边"最新样本"时刻不同）会带入 mm 级假差。
    t_q = node.max_stamp - 0.3 if node.max_stamp > 0.3 else None
    t_map_odom = node.lookup(args.map_frame, args.odom_frame, t_q)
    t_odom_base = node.lookup(args.odom_frame, args.base_frame, t_q)
    t_map_base = node.lookup(args.map_frame, args.base_frame, t_q)
    same_time = t_q is not None and None not in (t_map_odom, t_odom_base, t_map_base)
    if not same_time:  # 退化：取最新可用（此时判据放宽）
        t_map_odom = node.lookup(args.map_frame, args.odom_frame)
        t_odom_base = node.lookup(args.odom_frame, args.base_frame)
        t_map_base = node.lookup(args.map_frame, args.base_frame)

    print('--- B) tf2 查询（%s）---'
          % ('同一时刻 %.3f' % t_q if same_time else '各边取最新可用（判据放宽到 0.05）'))
    print('  %s→%s : %s' % (args.map_frame, args.odom_frame, '成功' if t_map_odom else '失败'))
    print('  %s→%s : %s' % (args.odom_frame, args.base_frame, '成功' if t_odom_base else '失败'))
    print('  %s→%s : %s' % (args.map_frame, args.base_frame, '成功' if t_map_base else '失败'))

    if args.mode == 'map_odom':
        if t_map_odom is None:
            print('  ✗ 查不到 %s→%s：定位还没出结果？还是查不到车上 %s→%s（看定位日志里的 [tf] 告警）？'
                  % (args.map_frame, args.odom_frame, args.odom_frame, args.base_frame))
            ok = False
        elif t_map_base is not None and t_odom_base is not None:
            diff = se3_diff(tf_to_se3(t_map_base), compose(tf_to_se3(t_map_odom), tf_to_se3(t_odom_base)))
            tol = 1e-6 if same_time else 0.05
            if diff < tol:
                print('  ✓ 合成自检通过：%s→%s = (%s→%s)·(%s→%s)，最大差 %.2e < %.0e'
                      % (args.map_frame, args.base_frame, args.map_frame, args.odom_frame,
                         args.odom_frame, args.base_frame, diff, tol))
            else:
                print('  ✗ 合成自检失败：%s→%s 与组合结果最大差 %.3e（应 < %.0e）'
                      % (args.map_frame, args.base_frame, diff, tol))
                ok = False
        else:
            print('  （%s→%s 或 %s→%s 查不到，跳过合成自检）'
                  % (args.map_frame, args.base_frame, args.odom_frame, args.base_frame))

    # --- C) 树连通：map→base_link 能解出来 -------------------------------------
    print('--- C) 整棵树是否连通（RViz/Nav2 的查法：%s→%s）---' % (args.map_frame, args.base_frame))
    if t_map_base is None:
        print('  ✗ 查不到 %s→%s：TF 树不连通（定位是否还没出结果？）' % (args.map_frame, args.base_frame))
        ok = False
    else:
        r, t = tf_to_se3(t_map_base)
        print('  ✓ 解析成功：t = [%.3f, %.3f, %.3f] m, yaw = %.2f°' % (t[0], t[1], t[2], yaw_of(r)))

    print('=== 结论: %s ===' % ('通过' if ok else '未通过'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
