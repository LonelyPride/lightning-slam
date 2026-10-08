#!/usr/bin/env bash
# =============================================================================
#  手动初始位姿一键发送（配合 run_loc_online 的 common.initial_pose_topic，默认 /initialpose）
#
#  典型用法（不需要 RViz，适合脚本化/远程操作）：
#    1) 车在图上 (12.30, -0.40)，车头朝 +x（yaw = 0°）：
#         bash send_initial_pose.sh --x 12.30 --y -0.40 --yaw 0
#    2) 车头朝 90°（逆时针）：
#         bash send_initial_pose.sh --x 12.30 --y -0.40 --yaw 90
#    3) 直接给四元数（x,y,z,w）：
#         bash send_initial_pose.sh --x 12.30 --y -0.40 --z 0 --quat 0,0,0.7071,0.7071
#    4) 把"定位此刻自己上报的车辆位姿"原样回灌（= 在 RViz 上车现在的位置点一下，
#       常用于验证功能是否正常、或车静止时做一次重新锁定）：
#         bash send_initial_pose.sh --from-tf
#
#  参数：
#    --x --y --z     位置(m，地图坐标系)；z 缺省 0
#    --yaw           车头朝向(度，绕 z 轴，逆时针为正)；与 --quat 二选一（都给了就用 --quat）
#    --quat          "qx,qy,qz,qw"（帧内的姿态，优先级高于 --yaw）
#    --topic         话题名，缺省 /initialpose（要与 config 里 common.initial_pose_topic 一致）
#    --from-tf       忽略 --x/--y/--yaw，改从 /tf 的 map→base_link 取定位当前上报的位姿
#    --hz            发布次数，缺省 1（RViz 是单次发布；万一丢包可用 3）
#
#  ★注意：
#    · frame_id 固定写 "map"（本工程地图帧名，见 localization_result.cc 的 map→base_link）；
#    · 位姿语义由 config 的 common.initial_pose_frame 决定：vehicle(默认)=车辆 base_link 系，
#      lidar=雷达系 T_map_lidar（与 --init_pose / 地图功能点 start 同语义）；
#    · 发送后定位会【立刻以该位姿为种子重新初始化】，日志出现
#      `[manual init] ...` → `init with external pose: ...`（被 min_init_confidence 拦下时会打印
#      `init confidence too low ... reject this init candidate`，此时重新发一次更准的即可）。
# =============================================================================
set -e

TOPIC=/initialpose
HZ=1
Z=0
YAW=""
QUAT=""
FROM_TF=0
X=""
Y=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --x) X="$2"; shift 2 ;;
        --y) Y="$2"; shift 2 ;;
        --z) Z="$2"; shift 2 ;;
        --yaw) YAW="$2"; shift 2 ;;
        --quat) QUAT="$2"; shift 2 ;;
        --topic) TOPIC="$2"; shift 2 ;;
        --hz) HZ="$2"; shift 2 ;;
        --from-tf) FROM_TF=1; shift ;;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "未知参数: $1（用 -h 看用法）"; exit 1 ;;
    esac
done

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$(pwd)/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$(pwd)/install/setup.bash"
fi

if [[ $FROM_TF -eq 1 ]]; then
    POSE=$(timeout 15 ros2 topic echo /tf --once | python3 -c '
import re, sys
txt = sys.stdin.read()
num = r"[-+0-9.eE]+"
tr = txt.split("translation:")[-1].split("rotation:")[0]
ro = txt.split("rotation:")[-1].split("covariance:")[0]
def pick(k, s):
    m = re.search(re.escape(k) + r"\s*:\s*(" + num + ")", s)
    return m.group(1) if m else None
v = [pick("x", tr), pick("y", tr), pick("z", tr), pick("x", ro), pick("y", ro), pick("z", ro), pick("w", ro)]
if any(i is None for i in v):
    sys.exit(1)
print(" ".join(v))
')
    read -r X Y Z QX QY QZ QW <<< "$POSE"
    echo "[send_initial_pose] --from-tf: 取到 map->base_link = ($X, $Y, $Z), q = ($QX, $QY, $QZ, $QW)"
elif [[ -n "$QUAT" ]]; then
    IFS=',' read -r QX QY QZ QW <<< "$QUAT"
else
    if [[ -z "$X" || -z "$Y" ]]; then
        echo "缺少 --x/--y（或用 --from-tf）"; exit 1
    fi
    if [[ -z "$YAW" ]]; then
        echo "缺少 --yaw 或 --quat"; exit 1
    fi
    read -r QX QY QZ QW <<< "$(python3 -c "
import math, sys
y = math.radians(float(sys.argv[1]))
print(0.0, 0.0, math.sin(y / 2.0), math.cos(y / 2.0))
" "$YAW")"
fi

echo "[send_initial_pose] topic=$TOPIC  pose: (x,y,z)=($X, $Y, $Z)  quat(x,y,z,w)=($QX, $QY, $QZ, $QW)"
for ((i = 0; i < HZ; i++)); do
    ros2 topic pub --once "$TOPIC" geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y, z: $Z}, \
      orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}}}"
done
echo "[send_initial_pose] 已发送；请在 run_loc_online 日志中确认 [manual init] / init with external pose"
