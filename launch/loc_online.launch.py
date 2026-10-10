#!/usr/bin/env python3
#
# 启动【在线定位】run_loc_online：加载已有地图 → 订阅实时点云/IMU → 发布 TF 与 2D 栅格图。
#
# ---------------------------------------------------------------------------
# 与 map_manager_3d 的对接约定（别改语义）：
#   map_manager_3d 的 set_mode{mode:"localization"} / switch_map 就是用这条命令启动定位进程的：
#       ros2 launch lightning loc_online.launch.py config:=<生成的 loc_<地图名>.yaml>
#   所以本文件必须提供名为 config 的启动参数（对应 gflags 的 --config），
#   并且【不能】额外传入 lightning 不认识的 gflags（否则 run_loc_online 会直接退出）。
# ---------------------------------------------------------------------------
#
# 用法：
#   ros2 launch lightning loc_online.launch.py                       # 用自动挑出来的基础配置
#   ros2 launch lightning loc_online.launch.py config:=/abs/loc_map01.yaml
#   ros2 launch lightning loc_online.launch.py init_pose:="1.2,0.5,0,0,0,0,1"
#
# 配置文件：
#   本包 config/ 下有两份可直接跑的基础配置（安装后在 <prefix>/lib/lightning/config/）：
#     gaoda.yaml            狗平台（Airy 96 竖装，含 2026-10-09 的 DR 时间戳/外参修复）
#     default_airy96.yaml   Airy 96 通用配置
#   下面 default_config 的挑选规则：优先 gaoda.yaml，找不到再用 default_airy96.yaml。
#   ★要改"定位哪张图"（system.map_path）时请复制一份再改，别改这两份基础配置本身。
#
# 启动参数：
#   config     定位配置文件路径（gflags --config）。默认见上。
#   init_pose  可选初始位姿，语义是【雷达系】T_map_lidar，格式 "x,y,z,qx,qy,qz,qw"（四元数 x y z w）。
#              留空（默认）= 不注入外部位姿，由地图 index.txt 的功能点 start 初始化（推荐）。
#              车没停在建图起点时才需要给；2D 位姿在运行中还能用 RViz 的 "2D Pose Estimate"
#              （话题 common.initial_pose_topic，默认 /initialpose，语义 = 车辆 base_link 在地图下的位姿）
#              随时重新初始化，不必重启本进程。
#
# 现场须知：
#   * 地图目录 = yaml 里的 system.map_path（如 ./data/map01/，相对【本进程 CWD】）；
#     一图一目录：index.txt + 分块 <id>.pcd 是 3D 定位必需件，map.pgm/map.yaml 供 2D 栅格发布。
#   * 定位端【不发布任何 Odometry 话题】：输出是 TF（map→odom）+ 2D 栅格话题
#     （yaml 的 system.publish_map.topic，gaoda.yaml 里是 /localization/map）。
#     所以"判断定位起来没有"要看 /tf 或该栅格话题——本平台没有 /chassis/lidar_odom。
#   * 需要 3D 窗口时改 yaml 的 system.with_ui（默认 false），远程/无 DISPLAY 环境保持 false。

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# 安装目录 lib/lightning/config/ 下的候选基础配置，按优先级排列
DEFAULT_CONFIG_CANDIDATES = ["gaoda.yaml", "default_airy96.yaml"]


def _config_dir():
    return os.path.join(get_package_prefix("lightning"), "lib", "lightning", "config")


def _default_config():
    """优先 gaoda.yaml（本平台在用的配置），否则退回 default_airy96.yaml。"""
    config_dir = _config_dir()
    for name in DEFAULT_CONFIG_CANDIDATES:
        path = os.path.join(config_dir, name)
        if os.path.exists(path):
            return path
    # 都不存在时返回首个候选：让 lightning 自己报"读不到配置"，比在这里静默失败更好排查
    return os.path.join(config_dir, DEFAULT_CONFIG_CANDIDATES[0])


def generate_launch_description():
    loc_exec = os.path.join(get_package_prefix("lightning"), "lib", "lightning", "run_loc_online")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=_default_config(),
        description="定位配置文件路径（gflags: --config）；map_manager_3d 会传生成的 loc_<地图名>.yaml",
    )

    init_pose_arg = DeclareLaunchArgument(
        "init_pose",
        default_value="",
        description='可选初始位姿（雷达系 T_map_lidar）"x,y,z,qx,qy,qz,qw"；留空=用地图功能点 start',
    )

    # 注意：空串也是合法取值（run_loc_online 判断 FLAGS_init_pose.empty() 后回退到功能点 start），
    #       所以这里固定带 --init_pose，不要用条件分支。
    loc = ExecuteProcess(
        cmd=[
            loc_exec,
            "--config", LaunchConfiguration("config"),
            "--init_pose", LaunchConfiguration("init_pose"),
        ],
        output="screen",
    )

    return LaunchDescription([config_arg, init_pose_arg, loc])
