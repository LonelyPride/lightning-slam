# Airy 96（速腾聚创 RoboSense）配置适配说明

> 本文档记录本次「为 lightning-slam 新增 Airy 96 激光雷达配置」的全部改动、取值依据、
> 需要现场核对的参数，以及实测验证结果。
> 参考对象：旧工程 lidar-slam 的 `config/custom.yaml`（同一台 Airy 96 平台）。

---

## 1. 目标

在 lightning-slam（干净版 lightning-lm 工程）中提供一个**可直接运行**的 96 线
速腾聚创 RoboSense Airy 雷达配置，用于：

- 建图：`run_slam_online` / `run_slam_offline`
- 定位：`run_loc_online` / `run_loc_offline`

---

## 2. 改动清单（本次只新增文件，未修改任何现有文件）

| 类型 | 文件 | 说明 |
| --- | --- | --- |
| 新增 | `config/default_airy96.yaml` | Airy 96 的完整配置（仅包含当前代码真正会读取的键） |
| 新增 | `doc/airy96_config_guide.md` | 本文档，记录改动内容、依据与注意事项 |

命名沿用本工程既有约定（`default_*.yaml`）。补充说明：当前源码包 `config/` 目录里**只有本次新增的
`default_airy96.yaml`**；其它 `default.yaml / default_robosense.yaml / default_livox.yaml / default_nclt.yaml /
default_utbm.yaml / default_vbr.yaml` 位于**已安装包**目录
`/home/h/ddh/code/ws/lightning_ws/install/lightning/lib/lightning/config/`
（其源头工程为 `/home/h/ddh/code/test/lightning-original_ws/src/lightning-lm/`）。

**未修改**：任何 `.cc/.h` 源码、CMake、其他 yaml 均保持原样。

---

## 3. 为什么现有代码路径可以直接适配 Airy 96

Airy 走的是本工程已有的 **RoboSense 分支**，不需要改代码：

| 环节 | 代码位置 | 结论 |
| --- | --- | --- |
| 雷达类型选择 | `src/core/lio/laser_mapping.cc:93-107`、`src/core/localization/localization.cpp:121-137` | `lidar_type: 4` → `LidarType::ROBOSENSE`，日志打印 `Using RoboSense Lidar`（注意：定位节点里 `localization.cpp:133` 的日志文本被误写成 `Using OUST 64 Lidar`，属既有笔误，不影响功能） |
| 点云解析 | `src/core/lio/pointcloud_preprocess.cc:134-172`（`RoboSenseHandler`） | `pcl::fromROSMsg` 到 `PointRobotSense`（`src/common/point_def.h:27-37`：`x,y,z`(float32) + `intensity`(float32) + `timestamp`(double, 秒)，**不含 ring**）。要求驱动点云至少包含 `timestamp` 字段，否则时间戳匹配不上（去畸变失效） |
| 去畸变时间戳 | 同上（`added_pt.time = (timestamp - header) * 1e3`） | RoboSense 用点自带**绝对秒**时间戳，代码已按此处理；`fasterlio.time_scale` 对 RoboSense **不生效**（只给 Velodyne 用） |
| 点结构定义 | `src/common/point_def.h`（`PointXYZIT`、`PointRobotSense`、`velodyne_ros::Point`、`ouster_ros::Point`、`FullPointType`）；`src/core/lio/pointcloud_preprocess.h:15`（`LidarType { AVIA=1, VELO32=2, OUST64=3, ROBOSENSE=4 }`） | 无需新增点类型；Airy 直接用已有的 `PointRobotSense` |
| 新增雷达类型？ | — | **不需要**：`lidar_type=4` 已是 RoboSense |

> 「96 线」本身在 RoboSense 分支里不参与逐线解算，`scan_line` 主要作为记录项/其他分支
> 使用（前端与定位端都会读取，必须存在）。

---

## 4. 本工程实际会读取的配置键（写入依据）

laser_mapping.cc 用 `yaml[...][...].as<T>()` **强转**读取，任何键缺失都会抛 `bad conversion`；
`YAML_IO::GetValue` 同样是 `as<T>()`，缺失同样抛异常。
下表是所有被读取的键（本次配置已全部包含）：

| 模块 | 读取位置 | 键 |
| --- | --- | --- |
| 前端 LIO | `src/core/lio/laser_mapping.cc:46-85` | `fasterlio.{max_iteration, esti_plane_threshold, filter_size_scan, filter_size_map, keep_first_imu_estimation, gyr_cov, acc_cov, b_gyr_cov, b_acc_cov, blind, time_scale, lidar_type, scan_line, point_filter_num, extrinsic_T, extrinsic_R, ivox_grid_resolution, ivox_nearby_type, use_aa, skip_lidar_num, kf_dis_th, kf_angle_th, enable_icp_part, min_pts, plane_icp_weight, imu_filter, proj_kfs}`、`roi.{height_max, height_min}` |
| SLAM 系统 | `src/core/system/slam.cc:32-36` | `system.{with_loop_closing, with_ui, with_2dui, with_g2p5, step_on_kf}`。注：栅格地图话题发布已改到**定位端**（2026-10-01），建图端不再有 `system.publish_map.*` |
| 2.5D 栅格图 | `src/core/g2p5/g2p5.cc:486-491` | `g2p5.{esti_floor, min_th_floor, max_th_floor, lidar_height, grid_map_resolution, floor_height}` |
| 回环检测 | `src/core/loop_closing/loop_closing.cc:48-53` | `loop_closing.{loop_kf_gap, min_id_interval, closest_id_th, max_range, ndt_score_th, with_height}` |
| 定位系统 | `src/core/system/loc_system.cc:26,33-35` | `system.map_path`、`common.{imu_topic, lidar_topic, livox_lidar_topic}`、**可选** `common.{initial_pose_topic, initial_pose_frame}`（手动初始位姿话题，见第 11.2 节）、**可选** `system.publish_map.*`（栅格地图话题，见第 11.3 节）、**可选** `system.tf.*`（TF 发布方式，见第 11.4 节）；以上可选键缺键时都用代码默认值 |
| 定位主流程 | `src/core/localization/localization.cpp:27,41-42,64-69,110-116` | `system.{enable_lidar_loc_skip, enable_lidar_loc_rviz, lidar_loc_skip_num, enable_lidar_odom_skip, lidar_odom_skip_num}`、`lidar_loc.{update_dynamic_cloud, force_2d, loc_on_kf}`、`fasterlio.{blind, time_scale, lidar_type, scan_line, point_filter_num}`、`roi.{height_max, height_min}` |
| 激光定位 | `src/core/localization/lidar_loc/lidar_loc.cc:61-99` | `maps.{with_dyn_area, max_pts_dyn_chunk, load_map_size, unload_map_size, delete_when_unload, load_dyn_cloud, save_dyn_when_quit, save_dyn_when_unload, dyn_cloud_policy}`、`lidar_loc.{update_kf_dis, update_lidar_loc_score, min_init_confidence, lidar_loc_odom_th, init_with_fp, enable_parking_static, enable_icp_adjust, try_self_extrap, grid_search_angle_step, grid_search_angle_range}`、**可选** `lidar_loc.{manual_init_rough_search, manual_init_yaw_search_deg, manual_init_yaw_search_step, manual_init_refine_iter}`（手动初始位姿兜底参数，见第 11.2 节）、`loop_closing.with_height` |
| 离线 bag 入口 | `src/app/run_slam_offline.cc:50-51`、`src/app/run_loc_offline.cc:43-44` | `common.{lidar_topic, imu_topic}` |

**当前代码不会读取的键**（lidar-slam 的 custom.yaml 里有，但本工程写了也不生效，故未写入）：
`pgo`、`ui`、`roi.range_max/range_min`、`system.pub_tf`、`lidar_loc.filter_z_min/max`、
`lidar_loc.filter_intensity_min/max`（`lidar_loc.cc:70-73` 已注释）、`time_offset`、
`gravity_obs_*`、`keyframe_in_base_frame`、`perception`、`motion_*_noise`、`loop_*_noise`、
`rk_loop_th`、`icp_fitness_th`、`min_overlap`、`nn_median_th` 等
（`src/common/params.cc` 里那套 `frontend_type / agi_sam / mapping` 参数的加载类，
在 `src/app/*.cc` 中**没有被使用**，属于遗留代码，与本配置无关）。

---

## 5. 关键取值来源（Airy 96）

| 参数 | 取值 | 来源 |
| --- | --- | --- |
| `fasterlio.lidar_type` | `4` (RoboSense) | 代码枚举，见第 3 节 |
| `fasterlio.scan_line` | `96` | Airy 96 规格 |
| `fasterlio.blind` | `0.1` | lidar-slam 实测值 |
| `fasterlio.point_filter_num` | `1`（可用 2~4 降负载） | lidar-slam 实测（原注释：96 线点云较密） |
| `fasterlio.filter_size_scan` / `filter_size_map` | `0.2` / `0.2` | lidar-slam 实测 |
| `fasterlio.ivox_grid_resolution` / `ivox_nearby_type` | `0.2` / `18` | lidar-slam 实测 |
| `fasterlio.esti_plane_threshold` | `0.3` | lidar-slam 实测（本工程默认 0.1） |
| `fasterlio.acc_cov / gyr_cov` | `0.01 / 0.01` | lidar-slam 实测（由 1e-4 放大，抑制“过度信任 IMU 短时预测”） |
| `fasterlio.b_acc_cov / b_gyr_cov` | `0.0005 / 0.0005` | lidar-slam 实测 |
| `fasterlio.keep_first_imu_estimation` | `true` | lidar-slam 实测 |
| `fasterlio.kf_dis_th / kf_angle_th` | `0.5 / 10.0` | lidar-slam 实测 |
| `fasterlio.enable_icp_part / plane_icp_weight` | `true / 700.0` | lidar-slam 实测 |
| `fasterlio.min_pts` | `50` | lidar-slam 实测 |
| `fasterlio.extrinsic_T / extrinsic_R` | `[0,0,0.004]` / `[0,-1,0; 1,0,0; 0,0,1]` | lidar-slam Airy 平台实测（雷达竖装，见第 6 节 ⚠） |
| `loop_closing.max_range` | `30.0` | lidar-slam 实测（本工程默认 20.0） |
| `lidar_loc.min_init_confidence` | `1.8` | lidar-slam 实测（本工程默认 0.8） |
| `lidar_loc.update_lidar_loc_score` | `1.5` | lidar-slam 实测 |
| `system.lidar_loc_skip_num / lidar_odom_skip_num` | `2 / 3` | lidar-slam 实测 |
| 其余未注明项 | — | 与本工程 `config/default.yaml` 默认值一致 |

---

## 6. ⚠ 投产前必须按本平台核对的 5 项

| # | 参数 | 为什么必须核对 |
| --- | --- | --- |
| 1 | `common.lidar_topic` = `/lidar/front/points_raw`、`common.imu_topic` = `/lidar/front/imu` | 沿用 lidar-slam 平台话题名。若实际驱动话题不同 ⇒ **收不到数据**（现象：节点正常但一直不出轨迹）。可用 `ros2 topic list` 核对后再改 |
| 2 | `fasterlio.extrinsic_T/R` = 雷达系→IMU 系 | 直接沿用 lidar-slam 的 **Airy 96 竖装（雷达 z=车体前进方向、y=上）+ IMU 竖装** 平台实测值。**若本平台安装姿态不同（如雷达水平安装），必须改为单位阵/对应外参**，否则 ESKF 传播与去畸变会逐帧累积十几度量级的方向误差。另：本工程**没有** lidar-slam 的“从 TF `base_link→laser_points_front` 自动注入外参”机制，配置文件里的值就是最终值 |
| 3 | `roi.height_max / height_min` = `60.0 / -1.0` | 本工程是**直接用雷达系 z** 与阈值比较（`pointcloud_preprocess.cc` 的 RoboSenseHandler 里没有 lidar-slam 的 `FilterUp`“上向参考系”变换）。雷达水平安装时 height 即真实高度；雷达竖装时这里裁的其实是**前向距离**，语义变化，需按实车确认 |
| 4 | `g2p5.floor_height / lidar_height` = `0.0 / 0.0` | 沿用 lidar-slam Airy 平台（关键帧点云为雷达系、地面 z≈0）。若雷达水平安装且离地 h，应设 `floor_height: -h`；`with_g2p5: false` 可先关掉本模块 |
| 5 | `system.map_path` = `./data/new_map/` | 相对路径依赖**进程启动目录**；建图端实际写到 `./data/<地图名>/`（`slam.cc:140`、`slam.cc:149`）。建议改成绝对路径，避免定位端找不到地图 |

---

## 7. 与 lidar-slam `config/custom.yaml` 的差异（未移植项及原因）

| lidar-slam 中的内容 | 本配置处理 | 原因 |
| --- | --- | --- |
| `pgo`（位姿图优化/回环权重） | 未写入 | 本工程暂未从 yaml 读取该段 |
| `ui`（车长车宽等） | 未写入 | 本工程暂未读取 |
| `roi.range_max/range_min`、`time_offset`、`gravity_obs_*`、`keyframe_in_base_frame` | 未写入 | 本工程代码未读取 |
| `lidar_loc.filter_z_*`、`filter_intensity_*` | 未写入 | 在 `lidar_loc.cc:70-73` 已被注释，不生效 |
| `motion_*_noise`、`loop_*_noise`、`rk_loop_th`、`icp_fitness_th`、`min_overlap`、`nn_median_th` | 未写入 | 本工程未从 yaml 读取（默认值在 `loop_closing.h`） |
| `perception`（感知发布、ring 输出、TF 广播） | 未写入 | 本工程还没有对应模块；`pointcloud_preprocess.h/.cc` 里**没有** `keep_ring_`/`ring_cloud_out_`/`FilterUp()`，即 Airy 的 ring 通道与“上向参考系高度过滤”目前都不存在 |
| `mask_roll_pitch / mask_z / recover_pose_path / update_err_*` | 未写入 | 本工程 `lidar_loc` 尚无这些功能 |
| `mapping`、`agi_sam`、`frontend_type` 等段 | 未写入 | 属于 `src/common/params.cc`（当前 app 未使用）的遗留解析 |
| `system.pub_tf`、`car_length` 等 | 未写入 | 本工程未从 yaml 读取（`loc_system.h` 里 `Options::pub_tf_` 是硬编码默认值 `true`）。**2026-10-01 起** TF 开关请用新增的 `system.tf.enable`（见第 11.4 节），`system.pub_tf` 依旧不生效 |

---

## 8. 使用方法

```bash
# 环境（本机已验证）
source /opt/ros/humble/setup.bash
source <ws>/install/setup.bash

cd <ws>/src/lightning-slam

# 在线建图（接实车 Airy 96）
./bin/run_slam_online  --config ./config/default_airy96.yaml

# 离线建图（录好的 bag）
./bin/run_slam_offline --config ./config/default_airy96.yaml --input_bag /path/to/airy.bag

# 在线定位（需已建好的地图，路径见 system.map_path）
./bin/run_loc_online   --config ./config/default_airy96.yaml

# 离线定位
./bin/run_loc_offline  --config ./config/default_airy96.yaml --input_bag /path/to/airy.bag
```

建图保存：在线模式调用 ROS2 服务 `lightning/save_map`（写 `./data/<地图名>/`）；
离线模式退出时自动保存。定位端读取的是 `system.map_path`。

### 8.1 可视化怎么开（Pangolin 3D UI / OpenCV 2D UI）

```yaml
system:
  with_ui: true      # 3D UI(Pangolin)：代码 slam.cc:46-52
  with_2dui: true    # 2D UI(OpenCV imshow)，需同时 with_g2p5: true，代码 slam.cc:66-77
```

要点：

1. **唯一开关就是 `system.with_ui`**：`slam.cc:33` 读入后覆盖 `slam.h:46` 的默认值 `true`；
   为 `false` 时 `slam.cc:46` 的分支整段跳过 —— 日志里**不会出现 `slam with 3D UI`**，
   `ui_` 保持 `nullptr`，`lio_->SetUI()`/`UpdateScan()`/`UpdateKF()` 全部空转，因此看不到窗口。
   本文件现已设为 `true`（实测日志出现 `slam.cc:47] slam with 3D UI`）。
2. **窗口只是容器**：只有收到 IMU/雷达数据触发 `UpdateScan/UpdateKF/UpdateNavState` 才会画出内容。
   没有数据进来时窗口是空的（不是报错）。
3. **需要 X 显示**：Pangolin 走 OpenGL，必须有 `DISPLAY`（本机 `DISPLAY=:0` 已验证可用，
   `./bin/test_ui` 可单独测窗口能否创建）。远程/容器内无显示时要么保持 `false`，要么转发 X11。
4. 定位端可视化另有开关：`system.enable_lidar_loc_rviz`（`localization.cpp:64-68`），二者相互独立。
5. 本节改动只涉及本配置文件，**未改任何源码**。

---

## 9. 实测验证记录（本机执行）

1. **YAML 语法**：`python3 -c "import yaml; yaml.safe_load(...)"` → 8 个顶层段解析通过
   （`common, fasterlio, roi, g2p5, system, loop_closing, lidar_loc, maps`）。
2. **键完整性**：用 yaml-cpp 写了复刻程序，按第 4 节表格逐条 `as<T>()` 读取本配置
   → `ALL KEYS OK`（`extrinsic_T`=3 元素、`extrinsic_R`=9 元素）。
3. **真实二进制启动**：
   ```bash
   timeout 12 ./bin/run_slam_online --config ./config/default_airy96.yaml
   ```
   日志：
   ```
   laser_mapping.cc:19] init laser mapping from ./config/default_airy96.yaml
   laser_mapping.cc:92] lidar_type 4
   laser_mapping.cc:104] Using RoboSense Lidar
   slam.cc:39] slam with loop closing
   loop_closing.cc:57] loop closing module is running in online mode
   slam.cc:47] slam with 3D UI
   slam.cc:82] online mode, creating ros2 node ...
   slam.cc:120] online slam node has been created.
   ```
   → 配置加载、LIO 初始化、回环、3D UI 窗口创建、ROS2 节点创建全部通过（进程被 `timeout` 主动结束，exit=124/143）。
   **注意**：本验证只覆盖“配置能被正确解析与初始化”，**不**代表没有实车数据时的精度表现。
4. **UI 可用性单独验证**：`timeout 6 ./bin/test_ui`（该程序只做 `PangolinWindow::Init()` + 空转），
   在本机 `DISPLAY=:0` 下正常创建窗口、无异常退出 ⇒ Pangolin/OpenGL 环境可用。

---

## 10. 顺带发现的既有问题（本次未修改，仅记录）

`default_robosense.yaml`（位于已安装包
`/home/h/ddh/code/ws/lightning_ws/install/lightning/lib/lightning/config/`，
源头工程 `/home/h/ddh/code/test/lightning-original_ws/src/lightning-lm/config/`，
`vbot` / `compare2` / `lighting-lm-comments` 等副本同样如此）
缺少两个键：`fasterlio.plane_icp_weight`、`fasterlio.proj_kfs`，
而 `laser_mapping.cc:81,85` 会强转读取它们。实测：

```bash
$ timeout 10 <该配置所在工程>/bin/run_slam_online --config ./config/default_robosense.yaml
E laser_mapping.cc:88] bad conversion
E slam.cc:27] failed to init lio module
E run_slam_online.cc:32] failed to init slam
```

⇒ 该配置在当前代码下**无法启动**（`grep` 对照：`default.yaml`/`default_nclt.yaml`/`default_livox.yaml`/
`default_vbr.yaml` 都有这两个键，只有 `default_robosense.yaml` 漏了）。
修复方式：把这两行补进该 yaml 即可（本次未擅自改动，等确认）。
注意：**本次新增的源码包内 `config/` 没有这个文件**，如需修复应在源头工程同步后再重新安装。

新增的 `config/default_airy96.yaml` 已包含这两个键（实测可启动）。

---

## 11. 源码修改记录：方案A（姿态输出统一到车辆 base_link 系）

> ⚠️ 本节记录的是**源码改动**（不是配置改动）。所有改动在代码注释里都以
> **「个人修改」**开头，可用 `grep -rn "个人修改" src/` 一次列全。
> 注：因本次在 `laser_mapping.*`、`localization.*`、`pangolin_window*` 中新增了注释与代码，
> 本文件前面章节里对这几个文件的行号引用可能会有几行漂移，按函数名/关键字检索即可。

**现象**：UI/日志里车的姿态像「绕 y 轴立起 90°、要上天」，用 roll/pitch/yaw 读姿态
时 pitch 恒在 ±90° 附近、真正的拐弯跑到 roll 上，±180° 附近乱跳。

**根因**（已用离线数据 + Sophus 源码 + 单元测试确认）：

1. ESKF/地图/世界系本身是**正确**的，不要动；问题只在「把 IMU 姿态直接当成车体姿态输出」。
   本平台 IMU 竖装：IMU 系 `x=向下, y=左, z=前`；车辆系 `x=前, y=左, z=上`，
   两者差一个固定的 `R_y(+90°)`。静止起步时 `R_wb(0)` 恰好就是这个 `M`，
   所以 UI 画出的车 x 轴指地 ⇒ 看起来立起来 90°。
2. 旧代码用 `SO3::angleX/angleY/angleZ` 读姿态，它们是 2×2 子块的 SVD log，**不是真正的 RPY**，
   在 IMU 竖装（车体姿态换算到 IMU 系后 pitch≈90°）时退化：实测静止时旧 `angleZ` 读到 -162°，
   而真实朝向只有几度。

**方案A 做法**（只在输出/显示边界换算，不动 ESKF）：令
`M = R_vehicle_imu = R_y(+90°)`，则 `T_w_vehicle = T_w_body * Mᵀ`；
姿态角改用标准 ZYX 欧拉角（`math::SE3ToRollPitchYaw`，内部 tf2 `getRPY`）。

| 边界 | 文件 | 改动 |
| --- | --- | --- |
| LIO 日志 | `src/core/lio/laser_mapping.cc:342-353` | 姿态换算到车辆系后打印 `rpy(deg)` |
| LIO → UI | `src/core/lio/laser_mapping.cc:206-223, 335-340` | NavState/画车姿态用车辆系；雷达系点云改用 `T_w_lidar = T_w_body * T_imu_lidar` |
| UI | `src/ui/pangolin_window.h/.cc`、`pangolin_window_impl.cc` | 新增 `UpdateBodyPose()` 专门给画车姿态；姿态显示改标准 RPY；`Reset()` 里补雷达外参 |
| 定位 → TF | `src/core/localization/localization_result.cc` `ToGeoMsg()` | `map→base_link` 发布真正的 base_link 姿态（原先发的是雷达系位姿） |
| 定位 → UI | `localization_result.cc` `ToNavState()`、`localization.cpp` | NavState/画车姿态换算到车辆系 |
| 换算工具 | `src/common/vehicle_frame.h`（新增，header-only，无需改 CMake） | `M`、`ToVehiclePose()`、`ToLidarPose()`、`ToVehiclePoseFromLidar()` 等 |

**换平台**：见下面 11.1 —— 已改为 yaml 配置（`common.vehicle_imu_mount` / `vehicle_imu_forward_axis` /
`vehicle_imu_R`），不必再改源码重编（注意该文件与 yaml 里的 `extrinsic_R/T`（雷达→IMU）不是同一件事，切勿混用）。

**对外行为变化**：`map→base_link` 的 TF 由「雷达系位姿」变为「真正的车辆(base_link)系位姿」，
下游若按老约定解析需要同步（Airy 竖装平台两者相差约 90°）。若 `T_imu_lidar_` 未设置（默认单位阵），
`ToGeoMsg()/ToNavState()` 的行为与修改前一致。

**验证**：整包编译 0 error；`vehicle_frame` 单元测试全通过（含静止、±179° 转向、
俯仰/侧倾、定位端换算、雷达系点云落图一致性）；用本仓库 Airy 配置离线跑一遍
`run_slam_offline`（`with_ui: false`）：日志 308 帧全部输出车辆系 rpy，
直线段 `rpy≈[0,0,ψ]`，把世界系速度投影到车辆系后**前进分量占绝对主导**（≈0.45 m/s，
左/上分量 <0.2 m/s），说明坐标系约定已一致。

### 11.1 方案A+：安装旋转不再写死（yaml 可配 + 可重力自动对齐）

**为什么还要再改一次**：方案A 的 `M` 是写死在 `common/vehicle_frame.h` 里的 `R_y(+90°)`，有三个问题：
① 换平台或换 IMU 安装方式（水平 / 倒装 / 垫片倾斜）必须改源码重编；
② 它是**全局**生效的常量，其它数据集的 IMU 本来就不是竖装，会被强行转 90°；
③ 实测本平台的"标称值"本身就有偏差（见下），写死值会让车一直带一个固定歪角。

**实测数据（本仓库真实 bag：`/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07`）**：

- 静止前 3 秒 `/lidar/front/imu` 平均比力（IMU 系，单位 g）= `[-0.9808, -0.0473, -0.1893]`，
  即真实"上"方向与标称的 `IMU -x` 相差 **11.25°**；
- 同一 bag 自带的 `/tf` 在起步时刻 `base_footprint` 的 `rpy = [0, 0, 180°]`（roll = pitch = 0），
  车体是水平的 ⇒ 这 11.25° 属于 **IMU 安装倾斜**，不是地面坡度；
- 因此方案A 的写死值会让车始终带着约 11° 的歪：日志静止段实测 `rpy = [-2.83, 11.20, 2.82]°`。

**配置方式（yaml 的 `common` 段，三项都可选；三项都不写 = 单位阵 = 不做换算）**：

| 键 | 取值 | 说明 |
| --- | --- | --- |
| `vehicle_imu_mount` | `auto`（推荐）/ `identity` / `x_down` / `x_up` | 车体安装方式；`auto` = 用 IMU 静止初始化测到的重力自动定"上"，对任意倾斜自动适应 |
| `vehicle_imu_forward_axis` | `x` / `y` / `z`（可带负号，默认 `z`） | 仅 `auto` 使用：车头方向对应 IMU 的哪个轴 |
| `vehicle_imu_R` | 行主序 9 个数 | 直接给出 `M`，优先级最高，可描述任意安装 |

`auto` 的原理：车辆 `+z` = 静止比力方向（与 `imu_processing.hpp::IMUInit()` 用的同一个 `mean_acc_`），
车辆 `+x` = 指定 IMU 轴在水平面内的投影，车辆 `+y = z × x`（右手系，指向左侧）。
标称竖装（`up = -x`、车头 `= z`）推出的结果恰好是 `R_y(+90°)`，与方案A 完全一致；
若所选轴与重力几乎平行（`|cos| > 0.98`）则**明确报错**并退化为不换算，不会瞎猜。

**代码落点**（均以「个人修改（方案A+）」开头）：

- `src/common/vehicle_frame.h`：新增 `SetVehicleImuRotation()`、`VehicleImuRotationFromName()`、
  `ImuAxisFromName()`、`VehicleImuRotationFromGravity()`；`R_vehicle_imu()` 改为读取运行期值（缺省单位阵）。
- `src/core/lio/laser_mapping.cc`：`LoadParamsFromYAML()` 末尾读取上述三个键（可选，缺失不影响启动）；
  `Run()` 每帧调用 `TryInitVehicleFrameFromGravity()`，IMU 首次初始化完成即定出 `M` 并打印矩阵。
- `src/core/lio/imu_processing.hpp`：新增只读接口 `GetMeanAcc()`。
- 定位端无需额外改动：`Localization` 内部持有 `LaserMapping`，走同一套代码路径、读同一份配置。
- 顺带补齐方案A 的一处漏点：`laser_mapping.cc` 中 IMU 高频（约 100 Hz）喂给 UI 的
  `ui_->UpdateNavState(kf_imu_.GetX())` 之前没有换算，会把车摆回 IMU 姿态，现已同样换算。

**与雷达安装姿态的关系（★本平台前提：雷达内置 IMU，两者是同一个刚体、永远一起安装）**：

- `M`（IMU→车辆）与"雷达的安装姿态"是**同一个未知量**，只差一个**单元内部固定常量**
  `C = fasterlio.extrinsic_R`（本平台 = `R_z(+90°)`，`extrinsic_T ≈ [0,0,0.004]` 也印证雷达相位中心与 IMU 基本重合）。
  逐轴自洽性：`R_z(90°)` 把 `lidar y → IMU -x`，而 yaml 注"雷达 y = 上" ⇒ **IMU x = 下**；
  把 `lidar z → IMU z`，而"雷达 z = 车体前进" ⇒ **IMU z = 前**，正是 `vehicle_imu_forward_axis: z`。
- 因此"雷达改水平 / 倾斜 / 倒装 / 再歪几度"= **`M` 变了**：只需改 `common.vehicle_imu_mount`
  （`auto` 连角度都不用给，代码不用动、不用重编）。
- `fasterlio.extrinsic_R/T` 是**单元内部**常量，**换安装不要动它**。若历史上曾用它去"凑"安装倾角，
  那它已被污染（把安装旋转混进了内部关系），重新安装后需要重新按单元内部关系标定，把安装部分交给 `M`。
- LIO 内核本来就不要求 IMU 竖直：`IMUInit()` 已把静止比力**显式对齐到世界系 +z**（`imu_processing.hpp`），
  任意安装姿态都成立；所以雷达重新安装不需要改 LIO 算法本体。

**随安装姿态确实会变、需要人工复核的参数（与 `M` 无关）**：

| 参数 | 是否随姿态变 | 说明 |
| --- | --- | --- |
| `roi.height_max / height_min` | **要改** | 本工程直接在**雷达系 z** 上过滤（`pointcloud_preprocess.cc:66/115/160`）。垂直安装时它裁的是**前向距离**（当前 60 / -1 的含义），改成水平安装后同一组数值变成"裁高度"，**语义翻转** |
| `g2p5.floor_height / lidar_height` | 只在**安装高度**变时改 | 二者是**世界系（z 朝上）**高度（`g2p5.cc` 里 `floor_coeffs_ = (0,0,1,-floor_height)`），姿态倾斜本身不影响；世界系已重力对齐，故与安装姿态解耦 |
| 地图 | 建议重建 | 世界系重力对齐 ⇒ 旧图几何上仍"通用"，但扫描面形态（垂直扇形 ↔ 水平环）改变，地面点分布/配准质量都会变 |
| 安装**平移**（雷达相位中心相对 base_link） | 目前**无参数** | `M` 只含旋转；换安装位置后输出的位置原点仍是雷达相位中心，若必须严格以 base_link 为原点需另加安装平移 |

单元测试里另对"雷达外参取垂直(90°) / 水平(0°) / 倾斜(30°) 三种"验证了 `T_w_lidar → T_w_vehicle`
数学结果一致 —— 这一条只说明**换算公式本身不依赖雷达外参**，不代表参数不用跟着改（见上表）。

**验证（本机执行）**：

- 单元测试（`g++ -std=c++17`，使用 production 的 `math::SE3ToRollPitchYaw`）全部通过，新增用例：
  缺省 = 单位阵、名称/轴名解析与非法值拒绝、重力对齐 = 标称值、
  **实测倾斜 up 下 auto 后静止倾斜 0.0000°（写死值残差 11.26°）**、轴与重力平行时报错、
  水平/倒装平台、以及雷达三种外参的解耦一致性。
- 真实 bag 离线跑 `run_slam_offline`（`with_ui: false`，`vehicle_imu_mount: auto`）：
  日志打印 `mean_acc(IMU 系) = -0.983177 -0.0485398 -0.194858` 与自动求得的 `R_vehicle_imu`；
  静止前 5 帧车辆系合成倾斜 **0.008°**（方案A 写死值同段为 **11.55°**），首帧 rpy 由
  `[-2.833, 11.198, 2.822]` 变为 `[-0.006, 0.001, 2.823]`（yaw 不变、歪角消失），308 帧输出均为车辆系姿态。

> 注：日志里的 `vel:` 是**世界系**速度（不是车辆系），其 z 分量由 LIO 的 z 漂移主导
> （实测均值约 0.11 m/s），所以不要用"速度上分量"来判断姿态对不对，看静止段 roll/pitch 更直接。

---

### 11.2 手动初始位姿 / 随时重新定位（2026-10-01 新增，详见报告第 12 章）

在线定位（`run_loc_online`）现在订阅 **`common.initial_pose_topic`**（默认 `/initialpose`，
即 RViz 工具栏 **"2D Pose Estimate"** 默认发布的话题，类型 `geometry_msgs/msg/PoseWithCovarianceStamped`）：
**运行中任意时刻**收到该消息，就以该位姿为种子**重新初始化**，用于手动校准车的初始位置。

| 新增配置键（均可选，缺键时用代码默认值 ⇒ 旧 yaml 也能跑新二进制） | 默认 | 说明 |
| --- | --- | --- |
| `common.initial_pose_topic` | `/initialpose` | 话题名；置 `""` 关闭该功能 |
| `common.initial_pose_frame` | `vehicle` | `vehicle` = 车辆 base_link 系位姿（**RViz "2D Pose Estimate" 的语义**，要求 RViz 的 Fixed Frame = `map`）；`lidar` = 雷达系 `T_map_lidar`（与 `--init_pose`、地图功能点 `start` 同语义） |
| `lidar_loc.manual_init_rough_search` | `true` | 手动种子被门槛拒掉时，是否用"粗分辨率(5 m) NDT + 小范围 yaw"再找一次种子 |
| `lidar_loc.manual_init_yaw_search_deg` / `_step` | `30.0` / `7` | 上述 yaw 搜索的范围(度)与采样数（7 ⇒ 步长 10°） |
| `lidar_loc.manual_init_refine_iter` | `2` | 接受手动种子后再迭代精修次数（消除"附近的局部最优"，使手动初值与正常跟踪的最优解一致） |

关键行为（已实测，详见报告第 12.6 节）：

- 车辆系位姿会**自动换算**成内部使用的雷达系 `T_map_lidar`
  （`T_map_lidar = T_map_vehicle * T_vehicle_imu * T_imu_lidar`，用 `common/vehicle_frame.h` 的现成工具）；
  直接按雷达系解释车辆位姿会整体差约 90°、被初始化门槛拒掉。
- 手动种子**排在功能点候选之前**，但仍要通过 `lidar_loc.min_init_confidence`（1.8）门槛；
  被拒会打印 `init confidence too low ... reject this init candidate`（**不会带飞定位**），
  重新点一次更准的即可。
- 启动时的默认初始化（地图 `index.txt` 的功能点 `start`）**不受影响**；离线定位 `run_loc_offline`
  没有话题入口，换初值请用 `run_loc_online --init_pose` 或改 `index.txt`。
- 命令行替代：`scripts/global_map_fix_check/send_initial_pose.sh`（`--x/--y/--z` + `--yaw`，
  或 `--quat`，或 `--from-tf` 回灌定位当前上报的位姿）。

---

### 11.3 栅格地图话题发布（给下游规划/控制，2026-10-01 新增，详见报告第 13 章）

**在线定位**（`run_loc_online`）现在可以把**已保存的栅格地图**按固定间隔**持续**发布成
`nav_msgs/msg/OccupancyGrid`（默认话题 **`/localization/map`**、默认 **1 Hz**），
下游规划/控制直接订阅即可，不必再自己解析 `map.pgm`。
发出去的就是**定位自己在用的那一份** `<map_path>/map.pgm` + `map.yaml`（两者同源 ⇒
"下游看到的地图 == 定位用的地图"）；文件被重新保存后**自动重载**，无需重启进程。

> 建图端 `run_slam_online` **不发布**这个话题（它只在建图期间存在，建完即退出）——
> 车长期运行时跑的是定位进程，由它当地图服务器更合理；建图端照旧只把地图存盘。

| 新增配置键（均可选，缺键时用代码默认值 ⇒ 旧 yaml 也能跑新二进制） | 默认 | 说明 |
| --- | --- | --- |
| `system.publish_map.enable` | `true` | 总开关；`false` 既不建发布者也不建线程 |
| `system.publish_map.topic` | `/localization/map` | 话题名 |
| `system.publish_map.interval` | `1.0` | 发布间隔(秒)；**下限保护** 0.05 s（配 0 会按 0.05 处理并告警） |
| `system.publish_map.path` | `""` | 栅格地图目录（含 `map.pgm`/`map.yaml`）；留空 ⇒ 复用 `system.map_path`（推荐留空，两者必然一致） |
| `system.publish_map.frame_id` | `map` | 消息 `header.frame_id`（本工程 SLAM/定位的世界系就是 `map`） |

关键行为（已实测，详见报告第 13.5 节）：

- **内容与 `map.pgm` 逐格一致**：读图逻辑是 `slam.cc:SaveMap` 写图逻辑的**严格逆变换**
  （像素→占据概率用 `negate=0` 的 map_server 语义 + `occupied_thresh`/`free_thresh` 阈值），
  行序按"图像第 0 行 = 地图顶部(y 最大)、OccupancyGrid 的 `data[0]` = 左下角(origin)"处理。
  实测：与 `map.pgm` 同格一致率 **1.0000**、占据↔空闲互翻 **0 格**；
  行序再用同期建图日志的 2376 个车辆轨迹点做独立判据：正确行序 **99.5%** 落"空闲"格，故意翻转后 **0.0%**。
- **只在文件变化时读盘**：按 `map.pgm` 的 mtime 判断，没变就发内存里的缓存
  （避免每秒一遍 1 MB 级别的读盘 + 解析）；重新保存地图后会自动重载并改发新图，无需重启。
- **可选键探测已加固**：本功能新增三层键 `system.publish_map.*` 时，暴露出 `YAML_IO::HasValue()`
  在"**整段配置缺失**"时会抛 `YAML::InvalidNode` 而崩（详见报告 13.6）；
  现已改为安全下钻（`IsDefined/IsNull → IsMap → operator[]`），并把**两层**版一并加固。
  因此"整段删掉 `publish_map`"的旧 yaml 也能正常启动（按默认值发布）。
  ⚠ 唯一约束：各键必须继续放在 `system:` 下（层级不能改）。
- **地图缺失不致命**：`<path>/map.pgm` 不存在时**只告警一次**（避免刷屏），不发布、不崩溃，
  定位本身照常工作；运行中把地图补齐后会自动开始发布。
- **不影响定位主流程**：发布走**独立线程**（每 `interval` 秒一帧），不占用 `rclcpp::spin` 线程 ——
  构造百万格消息比较费时，放 spin 里会挡住 IMU/点云回调、导致定位掉帧。
  线程在 `loc_->Init()` **之前**启动 ⇒ 地图还没加载完就已经在发，下游不必等定位收敛。
- **退出干净**：Ctrl-C 后发布线程在 50 ms 内响应退出并被 `join`（实测日志有 `[map pub] ... 线程退出`）。
- 下游用法示例：

  ```bash
  ros2 topic hz   /localization/map                       # 应为 ~1 Hz
  ros2 topic info /localization/map -v                    # 类型 nav_msgs/msg/OccupancyGrid
  ros2 topic echo /localization/map --once --field info   # 只看元数据（整条 echo 太大）
  ```

- 自检脚本（频率 + 元数据 + 与已保存地图逐格比对 + 用建图轨迹独立判定行序/朝向）：

  ```bash
  python3 scripts/global_map_fix_check/gs_map_topic_check.py --dur 6 \
          --ref data/new_map --ref-log /tmp/run_slam_online.INFO
  ```

### 11.4 TF 发布：定位改发 `map→odom`，与车上 TF 树合并（2026-10-01 新增，详见报告第 14 章）

**问题**：车上本来就有 `odom --(动态)--> base_footprint --(静态)--> base_link --(静态)--> laser_points_*/uwb_*`，
而本工程原来把定位结果**直接发成 `map→base_link`** ⇒ `base_link` 有两个父节点（`base_footprint` 与 `map`），
TF 树冲突（RViz/Nav2 报 multiple parents，画面上 map/odom 两棵树互相打架）。

**现在**：定位只补 **`map→odom`** 这一条边，先查车上已有的 `odom→base_link`（中间帧由 tf2 自动串联），
再按 `T_map_odom = T_map_base · (T_odom_base)⁻¹` 发布 ⇒ 得到完整无冲突的
`map → odom → base_footprint → base_link → ...`。
下游以 `map` 为 fixed frame 查 `base_link` 得到的**仍是定位位姿**（与旧行为数值等价）。

**配置**（`system.tf`，**全部可选**，整段删掉也能跑）：

| 键 | 默认 | 含义 |
| --- | --- | --- |
| `enable` | `true` | TF 总开关；`false` ⇒ 不发 TF、也不建 tf2 监听 |
| `mode` | `"map_odom"` | `map_odom` = 发 `map→odom`（需要车上已有 `odom→base_link`）；`map_base` = 老行为，直接发 `map→base_link`（会有双父节点，仅为兼容保留） |
| `map_frame_id` | `"map"` | 地图帧名（发布的父帧）。正常情况下定位结果自带 `frame_id="map"`，以结果里的为准；本项只在结果帧名为空时兜底 |
| `odom_frame_id` | `"odom"` | 里程计帧名（必须与车上 TF 一致；发布的子帧） |
| `base_frame_id` | `"base_link"` | 车体帧名（查 `odom→base_link` 的目标帧；定位结果 `child_frame_id` 为空时兜底） |
| `lookup_timeout` | `0.0` | 查 `odom→base_link` 的最长等待(秒)；默认不等待（回调在定位工作线程里，等久了会拖慢定位）；若日志频繁报"查不到 odom→base_link"可试 `0.02~0.05` |

**行为细节**（都对定位主流程无影响）：

- 查不到 `odom→base_link` 时**本帧不发** `map→odom`，并打 5 s 节流的 WARNING；**不会**退回
  `map→base_link`（那就是要修的冲突本身）。要回旧行为必须显式写 `mode: "map_base"`。
- 时标优先用**定位结果的时间戳**（tf2 插值），查不到则取"最新可用"。
- tf2 监听（`TransformListener`）自带**独立线程** spin `/tf`、`/tf_static`，不会与主节点
  `spin(node_)` 冲突，也不需要额外 spin。
- 自检脚本（父帧唯一性 + `map→base_link = (map→odom)·(odom→base_link)` 合成自检 + 整棵树连通）：

  ```bash
  python3 scripts/global_map_fix_check/gs_tf_tree_check.py --dur 8
  # 老行为对照：--mode map_base
  ```

---

## 12. 参考来源



- 旧工程配置：`/home/h/ddh/code/test/lidar-slam_ws/src/lidar-slam/config/custom.yaml`
- 旧工程外参/框架说明：`/home/h/ddh/code/test/lidar-slam_ws/src/lidar-slam/doc/framework_and_extrinsics_guide.md`
  （第 3 节：本平台「雷达垂直安装 + IMU 竖装」与外参 `T_imu_lidar` 的实测结论）
- 本工程参考配置（在已安装包目录 `/home/h/ddh/code/ws/lightning_ws/install/lightning/lib/lightning/config/`）：
  `default.yaml`、`default_robosense.yaml`、`default_livox.yaml`、`default_nclt.yaml`、`default_utbm.yaml`、`default_vbr.yaml`
  （源头：`/home/h/ddh/code/test/lightning-original_ws/src/lightning-lm/config/`）
- 本工程配置读取代码：
  `src/core/lio/laser_mapping.cc`、`src/core/lio/pointcloud_preprocess.cc`、`src/common/point_def.h`、
  `src/core/system/slam.cc`、`src/core/system/loc_system.cc`、`src/core/localization/localization.cpp`、
  `src/core/localization/lidar_loc/lidar_loc.cc`、`src/core/loop_closing/loop_closing.cc`、
  `src/core/g2p5/g2p5.cc`、`src/io/yaml_io.h`

> 后续如需把 lidar-slam 上已验证的增强能力（`FilterUp` 上向参考系高度过滤、ring 输出、
> TF 外参自动注入、定位端 `filter_z/intensity`、回环噪声参数可配等）移植到本工程，
> 需要改代码而不是改配置，届时另行出方案。


