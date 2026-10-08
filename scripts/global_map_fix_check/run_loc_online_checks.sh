#!/usr/bin/env bash
# =============================================================================
#  第三批修复（报告第 11 章）：**在线定位入口**验证
#
#  背景：`run_loc_online` 曾经无条件 `loc.SetInitPose(SE3())`（单位阵当 T_map_lidar），
#        而 `lidar_loc` 的初始化门槛又被 `if/else` 恒真分支短路，于是错误初值被无条件接受
#        → 用同一 bag 建的地图做在线定位也"完全不可用"。
#
#  本脚本跑两个对照（都用【修好后的】二进制，直接对应用户原始链路）：
#    A) 默认（不注入外部位姿）      —— 期望：`start loc without external init pose` →
#                                      `init with fp: start`，fitness_score > min_init_confidence
#    B) `--init_pose 0,0,0,0,0,0,1` —— 故意复刻修复前的单位阵初值，期望：被门槛拒绝
#                                      （`init confidence too low: ~0.62 <= 1.8`）→ 回退 `start` 成功
#
#  用法（默认即用户原始链路：三楼走廊 bag + 用户地图 + 默认 config）：
#    bash scripts/global_map_fix_check/run_loc_online_checks.sh
#    WS=/path/to/ws BAG=/path/to/bag CFG=/path/to/cfg RECORD=100 \
#      bash scripts/global_map_fix_check/run_loc_online_checks.sh
#
#  指标（跑完用 gs_loc_check.py 和建图日志比对定位精度）：
#    python3 scripts/global_map_fix_check/gs_loc_check.py \
#      --loc-log /tmp/loc_online_check_A.log \
#      --ref-log /tmp/run_slam_online.INFO --tag 'online-A'
# =============================================================================
# 注意：**不要**加 `set -u` —— ROS 的 setup.bash 里用到了未绑定变量（AMENT_TRACE_SETUP_FILES）。
set -x

WS=${WS:-/home/h/ddh/code/ws/lightning_ws}                    # 运行目录（决定 ./data/<地图>/）
CFG=${CFG:-$WS/src/lightning-slam/config/default_airy96.yaml} # 用户 config（map_path 相对本目录）
BAG=${BAG:-/home/h/ddh/bag/follow_car/20260914/decompressed/rosbag2_2026_09_15-10_22_04}
MAP=${MAP:-$WS/data/new_map}                                  # 仅用于打印，实际由 config 的 map_path 决定
RECORD=${RECORD:-100}                                         # 采样秒数（建图 bag 全长约 238 s）

cd "$WS"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

echo "map dir in use: $MAP  (chunks: $(ls "$MAP" 2>/dev/null | grep -c '\.pcd$'))"

for TAG in A B; do
    # ⚠ 不要用 `pkill -f run_loc_online`：本脚本自己的命令行里就含 "run_loc_online"（会自杀）。
    pkill -f 'install/lightning/lib/lightning/run_loc_online' 2>/dev/null   # 真正的可执行文件
    pkill -x run_loc_online 2>/dev/null                                     # 进程名精确匹配（保险）
    pkill -f 'ros2 run lightning run_loc_online' 2>/dev/null                # ros2 run 的 python 包装
    pkill -f 'ros2 bag play' 2>/dev/null
    sleep 2

    EXTRA=""
    [ "$TAG" = "B" ] && EXTRA="--init_pose 0,0,0,0,0,0,1"   # 复刻修复前的单位阵初值

    # shellcheck disable=SC2086
    nohup ros2 run lightning run_loc_online --config "$CFG" $EXTRA \
        > "/tmp/loc_online_check_$TAG.log" 2>&1 &
    sleep 8
    nohup ros2 bag play "$BAG" > "/tmp/loc_online_check_$TAG.bag.log" 2>&1 &
    sleep "$RECORD"
done

pkill -f 'install/lightning/lib/lightning/run_loc_online' 2>/dev/null
pkill -x run_loc_online 2>/dev/null
pkill -f 'ros2 run lightning run_loc_online' 2>/dev/null
pkill -f 'ros2 bag play' 2>/dev/null

for TAG in A B; do
    echo "===== [$TAG] 初始化 ====="
    grep -nE 'start loc without|use given init pose|init confidence too low|fitness_score is|init with (fp|external)' \
        "/tmp/loc_online_check_$TAG.log" | head -8
    echo "----- [$TAG] 定位分值 -----"
    grep -oE 'confidence: [0-9.]+' "/tmp/loc_online_check_$TAG.log" |
        awk '{s+=$2;n++;if($2<1.8)b++} END{printf "帧数=%d 均值=%.3f 低于1.8阈值=%d\n",n,s/n,b+0}'
done
