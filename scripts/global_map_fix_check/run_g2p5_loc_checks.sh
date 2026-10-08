#!/bin/bash
# 第二批修复（定位端 + g2p5）的复现脚本：跑 3 次建图 + 2 次定位
#
#   A) 建图：正确地面参数 (lidar_height=1.81, floor_height=-1.81)   -> /tmp/g2p5fix_run
#   B) 建图：与原对照相同的 0.0/0.0 参数（只隔离代码改动）          -> /tmp/g2p5fix_run_cfg0
#   L1) 定位：用修复后的 index.txt（start = 雷达系位姿）            -> /tmp/loc_fixrun
#   L2) 定位：把 index.txt 的 start 换回旧的 IMU 系位姿             -> /tmp/loc_oldstart_run
#
# 前置：补丁已编译（bin/run_slam_offline、bin/run_loc_offline）；
#       对照数据 /tmp/gs_fix_run（第一批修复时的运行目录）保持不动。
set -x
WS=/home/h/ddh/code/ws/lightning_ws/src/lightning-slam
BIN_SLAM=$WS/bin/run_slam_offline
BIN_LOC=$WS/bin/run_loc_offline
BAG=/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07
CFG_SRC=/tmp/gs_fix_run/airy96_final.yaml

rm -rf /tmp/g2p5fix_run /tmp/g2p5fix_run_cfg0 /tmp/loc_fixrun /tmp/loc_oldstart_run \
       /tmp/loc_map_fix /tmp/loc_map_oldstart
mkdir -p /tmp/g2p5fix_run /tmp/g2p5fix_run_cfg0 /tmp/loc_fixrun /tmp/loc_oldstart_run

# --- 生成两份仅 g2p5 地面参数不同的配置 ---
cp $CFG_SRC /tmp/g2p5fix_run/airy96_final.yaml
cp $CFG_SRC /tmp/g2p5fix_run_cfg0/airy96_final.yaml
python3 - <<'EOF'
import re
def patch(p, lh, fh):
    s = open(p).read()
    s2 = re.sub(r'lidar_height: [\d\.\-]+', 'lidar_height: %s' % lh, s, count=1)
    s2 = re.sub(r'floor_height: [\d\.\-]+', 'floor_height: %s' % fh, s2, count=1)
    assert s2 != s, p
    open(p, 'w').write(s2)
patch('/tmp/g2p5fix_run/airy96_final.yaml', '1.81', '-1.81')
patch('/tmp/g2p5fix_run_cfg0/airy96_final.yaml', '0.0', '0.0')
EOF

# --- A / B：建图 ---
cd /tmp/g2p5fix_run
$BIN_SLAM --config /tmp/g2p5fix_run/airy96_final.yaml --input_bag $BAG > stdout.log 2>&1
echo "runA_exit=$?"
cd /tmp/g2p5fix_run_cfg0
$BIN_SLAM --config /tmp/g2p5fix_run_cfg0/airy96_final.yaml --input_bag $BAG > stdout.log 2>&1
echo "runB_exit=$?"

# --- L1 / L2：定位（地图目录用 A 的产物；L2 只把 index.txt 的 start 行换回旧值） ---
cp -r /tmp/g2p5fix_run/data/new_map /tmp/loc_map_fix
cp -r /tmp/g2p5fix_run/data/new_map /tmp/loc_map_oldstart
sed -i 's/^start .*/start -0.000566496236975555924 0.000271782945616123092 0.000909618628254462835 -0.0381160767446231535 0.771781162654867736 -6.09031512600297458e-05 0.634744829010449707/' \
    /tmp/loc_map_oldstart/index.txt

cd /tmp/loc_fixrun
$BIN_LOC --config /tmp/g2p5fix_run/airy96_final.yaml --map_path /tmp/loc_map_fix/ --input_bag $BAG > stdout.log 2>&1
echo "L1_exit=$?"
cd /tmp/loc_oldstart_run
$BIN_LOC --config /tmp/g2p5fix_run/airy96_final.yaml --map_path /tmp/loc_map_oldstart/ --input_bag $BAG > stdout.log 2>&1
echo "L2_exit=$?"

echo "ALL DONE"
# 注：所有 run_* 程序在退出时都会命中 Pangolin/GLX 断言（EXIT=134），
#     这是【本机离线跑 UI 的既有现象】（对照 run /tmp/gs_fix_run 同样如此），
#     产物在此之前已全部落盘（日志里有 "map saved"/"dynamic maps saved"）。
