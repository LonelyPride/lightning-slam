#!/usr/bin/env bash
# =============================================================================
#  第五批（报告第 13 章）：**栅格地图话题发布**（默认 /localization/map）验证
#
#  被测功能：`run_loc_online`（**定位端**，长期运行的进程）把【已保存的栅格地图】
#            `<map_path>/map.pgm` + `map.yaml` 按固定间隔发成 nav_msgs/OccupancyGrid。
#            建图端 `run_slam_online` **不发布**（L8 回归）。
#
#  本脚本一次跑完 8 个用例（全程约 3~4 分钟）：
#    L1) 默认 config：频率 / 元数据 / 与 map.pgm 逐格一致 / 轨迹行序判据 / 干净退出
#    L2) `enable: false`：不建发布者不建线程、话题不存在、订阅 0 帧
#    L3) 旧 yaml（**整段删掉** publish_map）：不崩、按默认值发布（YAML_IO::HasValue 加固）
#    L4) `topic` / `interval` / `frame_id` 自定义：即时生效
#    L5) `path` 指向另一张地图（合成 10x10@0.1）：发的是那张图，而定位仍用 system.map_path
#    L6) 地图文件缺失：只告警一次、0 帧、进程存活、定位照常
#    L7) 与定位并行（同一 bag 0.5 倍速）：定位分值不退化 + 话题照发
#    L8) 回归：`run_slam_online` 无该话题、无发布日志
#
#  用法（默认即用户原始链路：用户地图 + 用户建图的 bag）：
#    bash scripts/global_map_fix_check/run_map_topic_checks.sh
#    WS=/path/to/ws CFG=/path/to/cfg BAG=/path/to/bag MAP=data/new_map \
#      BAGSEC=40 bash scripts/global_map_fix_check/run_map_topic_checks.sh
#
#  产物：变体 yaml / 日志全部落在 /tmp/map_topic_checks/，合成地图在 /tmp/map_topic_alt/。
#  行序判据需要一份含 `LIO state(vehicle/base_link)` 行的日志（建图或定位日志都行），
#  由 REFLOG 指定（默认 /tmp/run_slam_online.INFO）；文件不存在时该判据会自动跳过。
# =============================================================================
# 注意：**不要**加 `set -u` —— ROS 的 setup.bash 里用到了未绑定变量（AMENT_TRACE_SETUP_FILES）。
set -x

WS=${WS:-/home/h/ddh/code/ws/lightning_ws}                     # 运行目录（决定 ./data/<地图名>/）
CFG=${CFG:-$WS/src/lightning-slam/config/default_airy96.yaml}  # 用户 config（map_path 相对本目录）
BAG=${BAG:-/home/h/ddh/bag/follow_car/20260914/decompressed/rosbag2_2026_09_15-10_22_04}
MAP=${MAP:-data/new_map}                                       # 参考地图目录（相对 $WS）
REFLOG=${REFLOG:-/tmp/run_slam_online.INFO}                    # 行序判据用的轨迹日志（可缺）
BAGSEC=${BAGSEC:-40}                                           # L7 播放 bag 的秒数（定位要跑够才有分值统计）
T=/tmp/map_topic_checks                                        # 变体 yaml / 日志目录
ALT=/tmp/map_topic_alt                                         # L5 合成地图目录
MISS=/tmp/map_topic_missing                                    # L6 故意不存在的目录
D=$WS/src/lightning-slam/scripts/global_map_fix_check
CHK=$D/gs_map_topic_check.py

cd "$WS"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

rm -rf "$T" "$ALT" "$MISS"
mkdir -p "$T" "$ALT"


# --- 生成 L2~L6 的变体 yaml（只动 publish_map 块，其余与用户 config 完全一致）+ L5 合成地图 ---
python3 - "$CFG" "$T" "$ALT" <<'PY'
import os, re, sys
src, outdir, alt = sys.argv[1], sys.argv[2], sys.argv[3]
lines = open(src, encoding='utf-8').read().split('\n')

def block_end(i):
    """publish_map 块的结束行号（下一个缩进 <= 2 空格的键）。"""
    for j in range(i + 1, len(lines)):
        if re.match(r'^\S|^  \S', lines[j]):
            return j
    return len(lines)

def write(name, subs=(), drop_block=False):
    i0 = next(k for k, l in enumerate(lines) if re.match(r'^\s*publish_map:\s*$', l))
    i1 = block_end(i0)
    seq = [] if drop_block else lines[i0:i1]
    if not drop_block:
        for pat, rep in subs:
            hit = False
            for k, l in enumerate(seq):
                l2 = re.sub(pat, rep, l, count=1)
                if l2 != l:
                    seq[k], hit = l2, True
                    break
            assert hit, 'no match for %s' % pat        # 配置行改名时这里会立刻报错
    body = lines[:i0] + seq + lines[i1:]
    open(os.path.join(outdir, name), 'w', encoding='utf-8').write('\n'.join(body))

write('off.yaml',     [(r'^\s*enable:\s*true',      '    enable: false')])
write('custom.yaml',  [(r'^\s*topic:\s*"',          '    topic: "/map_custom"  #'),
                       (r'^\s*interval:\s*[\d.]+',  '    interval: 0.2'),
                       (r'^\s*frame_id:\s*"',       '    frame_id: "custom_map"  #')])
write('alt.yaml',     [(r'^\s*path:\s*""',          '    path: "%s"' % alt)])
write('missing.yaml', [(r'^\s*path:\s*""',          '    path: "%s"' % os.path.join(alt, 'no_such'))])
write('old.yaml', drop_block=True)                      # 旧 yaml：整段没有 publish_map

# L5 合成地图：10x10、0.1 m、origin(-5,-5)，左上 4x2 与右下 4x2 两块占据（0=黑=占据）
#   ⚠ 必须用 **P5 二进制**：C++ 侧的 cv::imread 与检查脚本的 read_pgm 都按 map_server 的
#     `map.pgm`（P5）格式读，写 ASCII 的 P2 会让两边都读不出来。
w = h = 10
pix = bytearray([254]) * (w * h)
for y0, x0 in ((0, 0), (8, 6)):
    for y in range(y0, y0 + 2):
        for x in range(x0, x0 + 4):
            pix[y * w + x] = 0
with open(os.path.join(alt, 'map.pgm'), 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (w, h))
    f.write(bytes(pix))
open(os.path.join(alt, 'map.yaml'), 'w', encoding='ascii').write(
    'image: map.pgm\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\n'
    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print('variant yamls ->', outdir, '; synthetic map ->', alt)
PY

# L8 用：关掉 3D UI（无 DISPLAY 也能起建图）
sed 's/^  with_ui: true/  with_ui: false/' "$CFG" > "$T/slam.yaml"
grep -q 'with_ui: false' "$T/slam.yaml"

echo "map dir in use: $WS/$MAP  (chunks: $(ls "$WS/$MAP" 2>/dev/null | grep -c '\.pcd$'))"

# --- 工具函数 ---------------------------------------------------------------
kill_all() {
    # ⚠ 不要用 `pkill -f run_loc_online`：本脚本自己的命令行里就含这个名字（会自杀）。
    pkill -f 'install/lightning/lib/lightning/run_loc_online' 2>/dev/null
    pkill -f 'install/lightning/lib/lightning/run_slam_online' 2>/dev/null
    pkill -x run_loc_online 2>/dev/null
    pkill -x run_slam_online 2>/dev/null
    pkill -f 'ros2 run lightning run_' 2>/dev/null
    pkill -f 'ros2 bag play' 2>/dev/null
    sleep 2
}

start_loc() {  # $1=yaml $2=日志；起定位并等它把节点/地图准备好
    nohup env GLOG_log_dir="$T" ros2 run lightning run_loc_online --config "$1" \
        > "$2" 2>&1 &
    sleep 8
}

sweep() {  # $1=话题 $2=秒数 $3=interval；其余参数透传给检查脚本
    local topic=$1 dur=$2 iv=$3
    shift 3
    python3 "$CHK" --topic "$topic" --dur "$dur" --interval "$iv" "$@"
    echo "检查脚本退出码 $? —— 0=通过 / 1=未收到消息或指标不符"
}

echo '===== L1 默认 config：频率/元数据/逐格一致/行序 ====='
kill_all
start_loc "$CFG" "$T/L1.log"
grep -m1 '栅格地图发布已开启' "$T/L1.log"
grep -m1 '已加载栅格地图' "$T/L1.log"
echo "topic list 命中数(应 >=1): $(ros2 topic list | grep -c 'localization/map')"
ros2 topic info /localization/map | head -3
REF=""
[ -f "$REFLOG" ] && REF="--ref-log $REFLOG"
# shellcheck disable=SC2086
sweep /localization/map 6 1.0 --ref "$MAP" $REF | tee "$T/L1.chk.log"

echo '===== L2 enable=false：不发、不建线程 ====='
kill_all
grep -c '栅格地图发布线程退出' "$T/L1.log"            # L1 进程被 SIGTERM 时应打印 1 行
start_loc "$T/off.yaml" "$T/L2.log"
grep -m1 '栅格地图发布已关闭' "$T/L2.log"
ros2 daemon stop > /dev/null 2>&1    # 清掉守护进程里上一轮定位端留下的话题缓存
sleep 2
echo "topic list 命中数(应 0): $(ros2 topic list | grep -c 'localization/map')"
sweep /localization/map 5 1.0 | tee "$T/L2.chk.log"
echo "定位进程存活(应 >=1): $(pgrep -c -f 'install/lightning/lib/lightning/run_loc_online')"

echo '===== L3 旧 yaml（整段缺 publish_map）：不崩、按默认值发 ====='
kill_all
start_loc "$T/old.yaml" "$T/L3.log"
echo "YAML::InvalidNode 行数(应 0): $(grep -c 'InvalidNode' "$T/L3.log")"
echo "定位节点是否起来(应 1): $(grep -c 'online loc node has been created' "$T/L3.log")"
sweep /localization/map 5 1.0 --ref "$MAP" | tee "$T/L3.chk.log"

echo '===== L4 topic/interval/frame_id 自定义 ====='
kill_all
start_loc "$T/custom.yaml" "$T/L4.log"
grep -m1 '栅格地图发布已开启' "$T/L4.log"              # 应显示 /map_custom / 0.2 s / custom_map
sweep /map_custom 4 0.2 --ref "$MAP" | tee "$T/L4.chk.log"

echo '===== L5 path 覆盖：发合成图 10x10@0.1，定位仍用 system.map_path ====='
kill_all
start_loc "$T/alt.yaml" "$T/L5.log"
grep -m1 '已加载栅格地图' "$T/L5.log"
grep -m1 'loaded chunks' "$T/L5.log"                  # 定位加载的仍是用户地图（合成图目录没有 *.pcd）
sweep /localization/map 5 1.0 --ref "$ALT" | tee "$T/L5.chk.log"

echo '===== L6 地图缺失：告警一次、0 帧、不崩 ====='
kill_all
start_loc "$T/missing.yaml" "$T/L6.log"
echo "告警条数(应 1): $(grep -c '未找到已保存的栅格地图' "$T/L6.log")"
sweep /localization/map 5 1.0 | tee "$T/L6.chk.log"
echo "定位进程存活(应 >=1): $(pgrep -c -f 'install/lightning/lib/lightning/run_loc_online')"
echo "定位节点是否起来(应 1): $(grep -c 'online loc node has been created' "$T/L6.log")"

echo '===== L7 与定位并行（同一 bag 0.5 倍速播放）====='
kill_all
start_loc "$CFG" "$T/L7.log"
nohup ros2 bag play --rate 0.5 "$BAG" > "$T/L7.bag.log" 2>&1 &
sleep "$BAGSEC"                       # 让定位先跑够（初始化 + 稳定出分值），期间发布线程一直在发
sweep /localization/map 6 1.0 --ref "$MAP" | tee "$T/L7.chk.log"
echo '----- 定位侧（应与修复后的行为一致：能初始化、分值不退化）-----'
grep -nE 'start loc without|fitness_score is|init with (fp|external)' "$T/L7.log" | head -5
grep -oE 'confidence: [0-9.]+' "$T/L7.log" |
    awk '{s+=$2;n++} END{printf "定位分值帧数=%d 均值=%.3f\n",n,s/n}'
pkill -f 'ros2 bag play' 2>/dev/null

echo '===== L8 回归：建图端不发布 ====='
kill_all
nohup env GLOG_log_dir="$T" ros2 run lightning run_slam_online --config "$T/slam.yaml" \
    > "$T/L8.log" 2>&1 &
sleep 12
echo "建图节点是否起来(应 1): $(grep -c 'online slam node has been created' "$T/L8.log")"
echo "发布相关日志行数(应 0): $(grep -cE '栅格地图发布|map pub' "$T/L8.log")"
ros2 daemon stop > /dev/null 2>&1    # 清掉守护进程里上一轮定位端留下的 publisher 缓存
sleep 2
ros2 topic info /localization/map | head -3           # 应为 Unknown topic（不是 count 0/1）
sweep /localization/map 5 1.0 | tee "$T/L8.chk.log"

kill_all
echo "全部残留进程(应 0): $(pgrep -c -f 'lightning/run_')"
echo "L1_L8_DONE （逐项输出见 $T/*.chk.log；判读见报告第 13.5 节）"
