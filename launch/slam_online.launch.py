#!/usr/bin/env python3
#
# 启动【在线建图】run_slam_online：LIO 前端 + 回环 + 2.5D 栅格，订阅实时话题，落盘由保存地图服务触发。
#
# ---------------------------------------------------------------------------
# 与 map_manager_3d 的对接约定（别改语义）：
#   map_manager_3d 的 set_mode{mode:"mapping"} 就是用这条命令启动建图进程的：
#       ros2 launch lightning slam_online.launch.py config:=<生成的 slam.yaml>
#   所以本文件必须提供名为 config 的启动参数（对应 gflags 的 --config），
#   并且【不能】额外传入 lightning 不认识的 gflags（run_slam_online 只有 --config 一个 flag，
#   多传任何东西都会让它直接退出）。
# ---------------------------------------------------------------------------
#
# 用法：
#   ros2 launch lightning slam_online.launch.py                     # 用自动挑出来的基础配置
#   ros2 launch lightning slam_online.launch.py config:=/abs/slam.yaml
#
# 配置文件：
#   本包 config/ 下有两份可直接跑的基础配置（安装后在 <prefix>/lib/lightning/config/）：
#     gaoda.yaml            狗平台（Airy 96 竖装，含 2026-10-09 的 DR 时间戳/外参修复）
#     default_airy96.yaml   Airy 96 通用配置
#   下面 default_config 的挑选规则：优先 gaoda.yaml，找不到再用 default_airy96.yaml。
#   ★map_manager_3d 生成的 slam.yaml 就是"复制一份基础配置 + 改几个键"，改完只需重新
#     set_mode{mapping} 生成一次，不需要动这里的默认值。
#
# 运行中保存地图（建图态才有这个服务）：
#   ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: map01}"
#   保存目录硬编码为 <建图进程 CWD>/data/<map_id>/（源码 slam.cc 里就是 "./data/" + map_id + "/"， 
#   【没有】map_root_path 之类的 yaml 键可配），日志会打印实际落点。
#   → 想让存图落进 map_manager_3d 的 MAPS_ROOT，必须让建图进程的 CWD = MAPS_ROOT 的父目录
#     （map_manager_3d 通过 chdir(lightning_cwd) 保证这一点，lightning_cwd 要指向 MAPS_ROOT 的父目录）。
#   注意 SaveMap 的返回码恒为 0，且会先清空同名目录再写盘，所以存图必须自己校验落盘结果。
#
# 现场须知：
#   * 建图端不发布 2D 栅格话题（那是定位端 run_loc_online 的活，见 yaml 的 system.publish_map），
#     且不发布任何 Odometry 话题；建图过程看 /tf 或点云话题。
#   * 需要 3D 窗口时改 yaml 的 system.with_ui（默认 false），远程/无 DISPLAY 环境保持 false。
#   * 建图/定位是两套独立程序、都发 map→odom：不要同时跑（map_manager_3d 会自动互斥）。

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
    slam_exec = os.path.join(get_package_prefix("lightning"), "lib", "lightning", "run_slam_online")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=_default_config(),
        description="建图配置文件路径（gflags: --config）；map_manager_3d 会传生成的 slam.yaml",
    )

    slam = ExecuteProcess(
        cmd=[slam_exec, "--config", LaunchConfiguration("config")],
        output="screen",
    )

    return LaunchDescription([config_arg, slam])
