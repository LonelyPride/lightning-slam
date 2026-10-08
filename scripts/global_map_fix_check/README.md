# global_map 外参补丁验证脚本

本目录存放「**雷达系点云 ↔ IMU 系位姿**混用」这一类缺陷的验证脚本，配套说明见
[`../../doc/global_map_extrinsic_fix_report.md`](../../doc/global_map_extrinsic_fix_report.md)：

- 第 1~9 章（`GetGlobalMap` 补丁）→ `gs_fix_ab.py` / `gs_plan_*.py` / `gs_layers.py` / `gs_ground2.py` / `gs_ground_check.py` / `gs_pcd_render.py`
- 第 10 章（定位端 + g2p5 补丁）→ `gs_g2p5_check.py` / `gs_projkfs_check.py` / `gs_loc_check.py` / `gs_loc_prior_check.py` / `gs_ground_height.py` / `run_g2p5_loc_checks.sh`
- 第 11 章（**在线定位入口**：初值坐标系 + 初始化门槛）→ `run_loc_online_checks.sh`（指标复用 `gs_loc_check.py`）
- 第 12 章（**手动初始位姿新功能**：RViz "2D Pose Estimate" → `/initialpose`）→ `send_initial_pose.sh`
- 第 13 章（**栅格地图话题发布**：`/localization/map`，供下游规划/控制）→ `gs_map_topic_check.py`
- 第 14 章（**TF 树**：定位改发 `map→odom`，与车上已有 TF 树合并）→ `gs_tf_tree_check.py`

脚本从验证时的 `/tmp` 工作目录原样归档；数据路径仍指向当时的运行产物
（`/tmp/planAplus_run/...` 补丁前、`/tmp/gs_fix_run/...` 第一批补丁后、
`/tmp/g2p5fix_run*`、`/tmp/loc_*` 第二批修复），换机器/换数据时用命令行参数或改文件头常量即可。

## 公共依赖（同目录）

| 文件 | 作用 |
| --- | --- |
| `airy_save_diag.py` | `read_pcd_xyz()`：读 `binary_compressed` PCD；另有旧版保存流程诊断工具 |
| `airy_ab_map.py` | `parse_log` / `parse_extrinsic` / `SE3` / `load_raw_frames` / `voxel` / 旋转工具，并用日志位姿独立重建地图 |

## 脚本一览

### 第一批：`GetGlobalMap` 外参补丁（报告第 1~9 章）

| 脚本 | 指标 | 与点密度无关？ | 结论 |
| --- | --- | --- | --- |
| `gs_fix_ab.py` | 新保存地图 ↔ 独立重建（FIX / BUG）最近邻距离，遍历 4 个帧对齐偏移 | ✅ 两版重建点数相同 | **决定性判据**：与 FIX 中位 0.014~0.019 m，与 BUG 0.096~0.100 m |
| `gs_plan_check.py` | 同一 0.2 m 体素重建后的局部平面度 / z 跨度 | ✅ 两版点数相同（1 112 704） | FIX 0.017 m vs BUG 0.038 m |
| `gs_plan_r.py` | 固定半径 0.8 m 邻域 PCA 平面度 | ✅ 同一物理体积同等采样 | FIX 重建 0.160 m vs BUG 重建 0.254 m |
| `gs_layers.py` | 2×2 m 网格内 z 间隙分层 → "地面叠层壳" | ✅ 只看层数不看点数 | 多层格占比 41.1 % vs 80.5 %，每格层数中位 1 vs 4 |
| `gs_pcd_render.py` | 俯视 + 侧视同视角渲染对比图 | — | `/tmp/gs_pcd_compare.png`（单层连续 vs 多层壳） |
| `gs_ground2.py` | 5×5 m 网格地面高度 → 全局平面残差 | ⚠ 只看两版**配对差** | FIX 0.499 m vs BUG 2.538 m（绝对值受真实坡道污染） |
| `gs_ground_check.py` | 固定 0.5 m z 窗口的"地面厚度" | — | 两版都饱和在 0.500 m，**无区分度，已弃用**（保留作记录） |

### 第二批：定位端 + g2p5 栅格图（报告第 10 章）

| 脚本 | 指标 | 结论 |
| --- | --- | --- |
| `gs_g2p5_check.py` | `map.pgm` ↔ 同次运行 `global.pcd` 的一致性（栅格范围外溢、占用格精度、障碍点完备性），可 `--render` 出对比图 | 栅格外溢 20.0/12.3 m → 0.9/1.8 m；障碍点覆盖率 41.6 % → **93.5 %**（中位 0.660 → 0.042 m） |
| `gs_projkfs_check.py` | `ProjectKFs` 投影点云 ↔ 当前帧实测点云 最近邻（FIX vs BUG） | NN 中位 **1.074 m → 0.153 m**（定位输入不再歪 90°） |
| `gs_loc_check.py` | `run_loc_offline` 的分值统计 + 定位位姿 ↔ LIO 轨迹误差 | L1（新 `index.txt`）分值中位 2.228、位置中位 **0.026 m**；L2（旧 `start`）分值中位 0.499、位置中位 1.445 m |
| `gs_loc_prior_check.py` | LO/DR「IMU 系增量 vs 雷达系增量」的先验差量级 | 平移差 ≈ **单帧位移的 53 %**（横向被换到垂向） |
| `gs_ground_height.py` | 世界系地面法向 + 雷达离地高度 h（用于配 `g2p5.lidar_height/floor_height`） | h 中位 **1.812 m**（5%~95 %: 1.62~2.16）；地面法向 z 中位 0.9665 |
| `run_g2p5_loc_checks.sh` | 一键复现：2 次建图（cfg 1.81 / cfg 0/0）+ 2 次定位（新/旧 `index.txt`） | 产物落在 `/tmp/g2p5fix_run*`、`/tmp/loc_*` |

### 第三批：在线定位入口（报告第 11 章）

| 脚本 | 指标 | 结论 |
| --- | --- | --- |
| `run_loc_online_checks.sh` | A) 用户原始链路（用户命令 + 用户建图的 bag + 用户地图）在线定位；B) 故意 `--init_pose 0,0,0,0,0,0,1` 复刻修复前的单位阵初值 | A：`start loc without external init pose` → `init with fp: start`，score **2.189**（>1.8）、均值 2.160；B：`init confidence too low: 0.678654 <= 1.8` 被拒（**与用户 20:28 失败日志的 0.678654 完全相同**）→ **回退 `start` 成功** |
| `gs_loc_check.py`（复用） | 在线日志同样适用：分值统计 + 定位位姿 ↔ 建图 LIO 轨迹最近邻 | 用户原始链路：分值中位 **2.035**（96 % > 1.8）、位置 XY 中位 **0.039 m**（p90 0.066、max 0.135） |

### 第四批：手动初始位姿（报告第 12 章）

| 脚本 | 指标 | 结论 |
| --- | --- | --- |
| `send_initial_pose.sh` | 向 `common.initial_pose_topic`（默认 `/initialpose`，即 RViz "2D Pose Estimate" 的话题）发一条 `PoseWithCovarianceStamped`：支持 `--x/--y/--z` + `--yaw`(度)、`--quat qx,qy,qz,qw`、`--from-tf`（回灌定位当前上报的 `map→base_link`，用于验证/重锁） | 实测两种方式均被定位接受（日志 `[manual init] ... → init with external pose`）；车在动时正确点击：接受 1.90959 → 精修 **2.04647**，位姿收敛 +0.68 m（点击延迟），拉回后跟踪分值 median **2.106**（同路段无干预参照 1.993）、0 次低分告警 |

### 第五批：栅格地图话题发布（报告第 13 章）

| 脚本 | 指标 | 结论 |
| --- | --- | --- |
| `gs_map_topic_check.py` | 订阅 `system.publish_map.topic`（默认 `/localization/map`，由 **`run_loc_online`** 发布）N 秒：帧数/频率/首帧延迟、`info` 元数据、与已保存 `map.pgm`+`map.yaml` 的**逐格一致率**与"占据↔空闲互翻"格数；`--ref-log` 再用日志里的车辆轨迹做**行序/朝向**独立判据 | **定位端（现方案）** 默认 config：**0.998 Hz**、元数据与 `map.yaml` 逐项一致、119.6 万格**同格一致率 1.0000 / 互翻 0 格**、轨迹 99.5% 落空闲格（故意翻转后 0.0%）；`enable=false` 不发；旧 yaml（缺该段）走默认值 0.999~1.002 Hz 一致；`topic/interval/frame_id` 改配置即时生效（4.982 / 5.169 Hz）；`path` 指向合成图（10×10@0.1，P5）**一致率 1.0000**；地图缺失只告警 1 次 0 帧不崩；与定位并行（bag 0.5×，40 s）**0.996~0.999 Hz** 且定位分值无退化（均值 2.162 / 2.143）；**建图端 `run_slam_online` 无该话题**（`Unknown topic`、订阅 0 帧） |
| `run_map_topic_checks.sh` | 一键跑完上述 **L1~L8** 八个用例（自动生成变体 yaml + 合成地图；日志/变体在 `/tmp/map_topic_checks/`、合成地图在 `/tmp/map_topic_alt/`） | 逐项判据与手工一致：L1/L3/L4/L5/L7 检查脚本**退出码 0**，L2/L6/L8 **退出码 1 属预期**（验证的正是"0 帧/无话题"）；L8 建图端 0 发布日志 + `Unknown topic`；结尾 `残留进程(应 0): 0` → `L1_L8_DONE` |

### 第六批：TF 树（报告第 14 章）

| 脚本 | 指标 | 结论 |
| --- | --- | --- |
| `gs_tf_tree_check.py` | 订阅 `/tf` + `/tf_static` N 秒统计**每个 child frame 的父帧集合**，并用 tf2 查询 + 合成自检：A) `base_link` 父帧必须**唯一且不是 `map`**；B) `map→odom` 存在且 `map→base_link ≟ (map→odom)·(odom→base_link)`（同一时刻查询，容差 1e-6）；C) 整棵树 `map→base_link` 能解析（RViz/Nav2 的查法） | **默认（`system.tf.mode=map_odom`，用户 bag 27 s + 用户地图）**：`odom <- map`、`base_footprint <- odom`、`base_link <- base_footprint`、`laser_points_*/uwb_* <- base_link`；A 父帧 `['base_footprint']` 唯一 ✓；B 合成最大差 **3.33e-16** ✓；C 解析成功 `t = (-0.049,-0.047,0.047) m, yaw 6.61°` ✓ → **退出码 0**。**反证**（手工再发一条 `map→base_link`）：报 `✗ 双父节点` + `✗ 查不到 map→odom` → 退出码 1，说明判据能抓住旧行为。**兼容回归 `mode=map_base`**：父帧 `base_link <- base_footprint, map`（= 老行为），日志有 map_base 警告且**不建 tf2 监听**，`--mode map_base` 下退出码 0 |

## 用法

```bash
D=scripts/global_map_fix_check

# 第一批
python3 $D/gs_fix_ab.py --offs 0,1,2,3   # 决定性判据（较慢：重建 2.9M 点 ×2）
python3 $D/gs_plan_check.py              # 等密度平面度
python3 $D/gs_plan_r.py                  # 固定半径平面度
python3 $D/gs_layers.py                  # 叠层壳（快，秒级）
python3 $D/gs_ground2.py                 # 地面成面（快，秒级）
python3 $D/gs_pcd_render.py              # 目视对比图（快，秒级）

# 第二批（先跑 runner，再出指标；详述见报告 10.5）
bash $D/run_g2p5_loc_checks.sh
python3 $D/gs_g2p5_check.py --run /tmp/g2p5fix_run --tag 'new-code cfg 1.81' --render /tmp/gs_g2p5_new.png
python3 $D/gs_projkfs_check.py
python3 $D/gs_loc_check.py --loc-log /tmp/loc_fixrun/stdout.log --tag 'L1'
python3 $D/gs_loc_prior_check.py
python3 $D/gs_ground_height.py

# 第三批（在线定位入口，报告 11.5）：A=用户原始链路，B=故意给单位阵初值
bash $D/run_loc_online_checks.sh
python3 $D/gs_loc_check.py --loc-log /tmp/loc_online_check_A.log \
        --ref-log /tmp/run_slam_online.INFO --tag 'online-A'

# 第四批（手动初始位姿，报告第 12 章）：run_loc_online 跑起来后随时发
bash $D/send_initial_pose.sh --x 12.30 --y -0.40 --yaw 0    # 位置 + 车头朝向(度)
bash $D/send_initial_pose.sh --from-tf                      # 回灌定位当前上报的位姿

# 第五批（栅格地图话题发布，报告第 13 章）
bash $D/run_map_topic_checks.sh                                            # 一键 L1~L8（会自动生成变体配置与合成地图）
# 或手动：run_loc_online 跑起来后订阅检查
python3 $D/gs_map_topic_check.py --dur 6                                    # 只查频率/元数据
python3 $D/gs_map_topic_check.py --dur 6 --ref data/new_map \               # + 与已保存地图逐格比
        --ref-log /tmp/run_slam_online.INFO                                 # + 轨迹行序判据（任一含轨迹的 INFO 日志）

# 第六批（TF 树，报告第 14 章）：run_loc_online 与车上 TF 都在跑时
python3 $D/gs_tf_tree_check.py --dur 8                                      # 期望：base_link 父帧唯一 + map→odom 合成自检通过
python3 $D/gs_tf_tree_check.py --dur 8 --mode map_base                      # 老行为（直接发 map→base_link）的对照
python3 $D/gs_tf_tree_check.py --dur 8 --odom-frame odom_combined           # 车上帧名不同时按实际填
```

`gs_fix_ab.py` 可用参数覆盖路径：
`--log --yaml --bag --saved --old-bug --topic --offs`；
`gs_g2p5_check.py` 支持 `--run --tag --log --render`；
`gs_loc_check.py` 支持 `--loc-log --ref-log --tag`；
`gs_map_topic_check.py` 支持 `--topic --dur --interval --ref --ref-log --min-agree --dump`；
`run_loc_online_checks.sh` 支持环境变量覆盖 `WS / CFG / BAG / MAP / RECORD`；
`run_map_topic_checks.sh` 支持环境变量覆盖 `WS / CFG / BAG / MAP / REFLOG / BAGSEC`（`BAGSEC` = L7 播包秒数，默认 40）；
`gs_tf_tree_check.py` 支持 `--dur --map-frame --odom-frame --base-frame --mode`。

> ⚠ `run_map_topic_checks.sh` 的用法要点（都是踩过的坑）：
> 1. **L5 合成地图必须是 P5 二进制 PGM**：C++ 侧 `cv::imread` 与 `gs_map_topic_check.py` 的
>    `read_pgm()`（`assert magic == b'P5'`）都按 map_server 的 `map.pgm` 格式读，写 ASCII 的 `P2`
>    会让检查脚本直接断言失败；
> 2. **查 `ros2 topic list` 前先 `ros2 daemon stop`**：守护进程会缓存上一轮注册过的话题名，
>    不清缓存时 `enable=false` / 建图端回归会看到"假阳性"话题；
> 3. 清理进程用 `pkill -f 'install/lightning/lib/lightning/run_loc_online'` 这类**具体路径**，
>    不要用 `pkill -f run_loc_online` —— 脚本自身命令行里含该字符串，会自杀。
> 4. **自己写 TF 检查脚本时，`/tf_static` 必须用 `transient_local` QoS**：静态 TF 只在启动时发一两条，
>    晚加入且用 volatile 的订阅者**一条都收不到**（`gs_tf_tree_check.py` 初版就踩了：
>    只看到 `odom <- map`、看不到 `base_link <- base_footprint`，但 tf2 查询却是通的）。
> 5. **TF 的"同一时刻"比较要显式指定时间戳**：三条边各自取"最新可用"时，样本时刻不同会带入 mm 级假差
>    （初版得到 5.06e-03 的"失败"）。`gs_tf_tree_check.py` 现在统一用
>    `t = 最新TF时间 - 0.3 s` 查询，容差才能收到 1e-6。

> ⚠ 本工程产物的 **md5 不可用于判同**：点结构 `PointXYZIT` 含未初始化的对齐填充字节，
> 每次运行的 PCD 字节都不同（几何逐点相同）。判同请用点坐标比较（见报告 10.3-(1)）。

## 运行期 UI 截图（辅助，非必须）

`ffmpeg -f x11grab` 抓 root 在本环境全黑（`run_slam_offline` 仅 ~7.4 s），
改用窗口级抓取：

```bash
WID=$(xwininfo -root -tree | grep '"UI"' | head -1 | awk '{print $1}')
xwd -id "$WID" -silent > ui.xwd && ffmpeg -i ui.xwd -y ui.png
```

## 运行期 UI 截图（辅助，非必须）

`ffmpeg -f x11grab` 抓 root 在本环境全黑（`run_slam_offline` 仅 ~7.4 s），
改用窗口级抓取：

```bash
WID=$(xwininfo -root -tree | grep '"UI"' | head -1 | awk '{print $1}')
xwd -id "$WID" -silent > ui.xwd && ffmpeg -i ui.xwd -y ui.png
```
