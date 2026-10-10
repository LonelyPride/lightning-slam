# dog_platform_frame_check —— 狗平台"坐标系/方向"的验收脚本

配套报告：[`../../doc/dog_platform_frame_fix_report.md`](../../doc/dog_platform_frame_fix_report.md)
（**先读它的 §0 勘误**）。

> **★ 重要（2026-10-09 第 7 次修复）**：本目录的脚本**都能用**，但第 6 次修复时用它们得出的几条**结论是错的**
> —— `extrinsic_R` **不该**改成单位阵、`imu_acc_up_sign` **不该**取 −1（会 ⇒ 地图**重影**，厚度中位 115 mm vs 正确的 50.7 mm）。
> 错在**方法**：拿【雷达系】的量（`up_laser`、地面法向）**直接**和【IMU 系】的加计均值 `â` 比角度，缺一次 `extrinsic_R` 换算；
> 乘上 `extrinsic_R` 再比 ⇒ 只差 **0.77°**（⇒ `â` 是物理"上"⇒ 符号 `+1`）。
> 表里三条"≈180°/179.23°/137.877°"都是这么来的假值（其中 137.877° 的真凶是当时 `ground_normal_lidar` 还是 val 平台的值）。
> 详见 `doc/modification_log.md` §4.1/§4.2。**改外参后必须量重影**（下面两个新脚本）。

本目录脚本从验证时的 `/tmp` 工作目录**原样归档**（原来叫 `final_check6.py` / `map6_check.py` /
`floor_probe.py` / `qb_plane_probe.py`）；数据路径常量仍指向当时的运行产物（bag、日志、地图目录），
换机器/换 bag 时改文件头常量即可。**所有判据都锚定在 bag 的 `/tf_static`、bag 的 IMU、点云几何上，
不依赖 URDF 文本，也不依赖"地图看着正不正常"**（这次 bug 恰恰是"地图看着正常"）。

| 脚本 | 用途 | 判据 / 结论 |
| --- | --- | --- |
| `frame_ground_check.py` ★先跑这个 | **A)** 自行串 bag 的 `/tf_static` 链 `base_link→rslidartmp1→laser_points_front`，算"车体上方"在雷达系(=IMU 系)的表达 `up_laser`；取 bag 前 233 个 IMU 采样（= 本次运行的静止初始化窗口）的比力均值 `â`；<br>**A2)** 读运行日志里 `R_vehicle_imu` 第 3 行，与 `up_laser` 对比；<br>**B)** 新旧两份地图在**起点**处 RANSAC 拟合地面（3 m 半径），报"传感器离地高度"与"地面法向 vs 竖直" | `angle(up_laser, â) = 179.51°` **✗ 假值**（跨坐标系）；正确写法是先乘 `extrinsic_R`：`R_m·up_laser` vs `â` ⇒ **0.77°** ⇒ 符号 `+1`。地图起点离地 **0.503 m**（≈ `g2p5.lidar_height` 0.46）**✓ 有效** |
| `map_frame_check.py` | 用工程自带函数（`airy_ab_map.parse_log` / `read_pcd_xyz`，逻辑同 `gs_ground_height.py`）逐项对比**新旧两份地图**：`R_vehicle_imu` 第 3 行、`base_link rpy`、世界观下地面法向 z 分量、离地高度 | 车体 `yaw` 中位 **−134.46° → +46.23°**：**✓ 测量有效，但解释要改** —— 那 180.69° 是**车辆系朝向**（`forward_axis: z` + 倒扣），**不是世界系翻转**（实测新图与旧图最近邻中位 4 mm）；地面法向 z 分量 ≈1 ✓ |
| `floor_probe.py` | 只读探测：6 帧点云 RANSAC 出"离传感器 0.15~0.9 m 的最大平面"，**符号由"传感器在平面哪一侧"唯一确定**（与加计无关）⇒ 独立判定"加计比力朝上还是朝下"；再与 tf_static 链对照 | 地面 **0.4862 m**、内点 99.5%、`angle(上_地面, â) = 179.23°` **✗ 假值**（同上，缺 `extrinsic_R` 换算；换算后 0.77°）；`angle(up_urdf, 上_地面) = 0.42°` ✓ 有效 ⇒ `ground_normal_lidar = [-0.6699,-0.6601,-0.3399]` 就是这么测的（**这一项是对的、本次保留**） |
| `yband_plane_probe.py` | 反证用：`vehicle_tilt_and_ground_check.py` 的 [Q-B] 在狗平台上报的"离地高 0.125 m、地面法向≈+y"**是假的** | 取 `|y−0.125|<0.05` 的点（10.0% 的点），它们到**真实地面平面**的距离中位 **−0.0001 m**（90.7% <3 cm）⇒ 本来就躺在地面上；只拿这条薄带做 SVD 会把法向拉成 ≈`+y`（与真实地面法向差 **48.47°**）⇒ **薄带拟合病态**，不是另一块面 |
| **`ghost_metrics.py`** ★第 7 次修复新增（决定性） | **量"重影/叠层"**：① 每点邻域(0.6 m) PCA 最小特征值开方 = **表面厚度**(mm)；② 邻域点投到局部法向后做"间隙 >3 cm 的聚类"，有 ≥2 个质量各 ≥15% 的簇 ⇒ 记"**双面**"；③ 地面 2×2 m 格内沿 z 按 0.3 m 分层（工程 `gs_layers.py` 思路） | 正确外参 **50.7 mm / 双面 0.7%**；第 6 次修复的单位阵 **115 mm / 8.9%**（≈ 差一倍、`extrinsic_R` 用错 ⇒ 去畸变沿错轴抹开）。**只看"地图像不像"抓不到**，所以这是改外参后必须跑的一条 |
| **`map_orientation_check.py`** ★第 7 次修复新增 | 判**世界系 z 是否物理朝上**：起点(原点) 3 m 内取点，z∈[−1.5,1.5] 做 5 cm 直方图取最密 bin 当地面 ⇒ 报"传感器离地 = −z_floor" | 本平台正常 ≈ **+0.525 m（z 朝上 ✓）**；若报负值 ⇒ 整图上下颠倒（本次第 7 次修复实测 +0.525 m、z 跨度 6.4 m ✓） |

> 后两个脚本是第 7 次修复时新写的（原 `/tmp/e1/metrics.py`、`/tmp/e1/orient.py`），**都支持直接传 pcd 路径**：
> `python3 scripts/dog_platform_frame_check/ghost_metrics.py /tmp/e1/fix7/data/new_map/global.pcd /tmp/e1/R_I_sm1/data/new_map/global.pcd`
> （标签取该路径往上第三级目录名，便于多份变体一次对比）。
> 另外"**地图基准有没有变**"只需把两份 `global.pcd` 直接比最近邻、再把其中一份绕 z 转 180° 比一次，
> 内联命令见 `doc/modification_log.md` §4.2 的复现块第 3 步（本次实测：第 7 次修复图 vs 第 6 次修复前在线图 = 4 mm ⇒ 同基准）。

## 公共依赖

- `scripts/global_map_fix_check/airy_ab_map.py`（`parse_log` / `read_pcd_xyz`）——本目录脚本用相对路径导入，
  不再写死 `/home/...`，所以整仓库搬走也能跑。
- `rosbag2_py` + `rclpy` + `sensor_msgs` + `tf2_msgs`（ROS 2 humble）+ numpy。

## 用法

```bash
cd /home/h/ddh/code/ws/lightning_ws/src/lightning-slam

# 0) 前置：用修好后的配置跑一遍离线建图（把这里产出的日志/地图填进下面脚本的常量）
#    ros2 run lightning run_slam_offline --config config/gaoda.yaml
#    注：脚本章节 5/6 若不传参数，默认读 <仓库根>/data/new_map/global.pcd
#    （= 在工程根跑 run_slam_offline 时 system.map_path 的产出；没跑过会提示"文件不存在"，属正常）

# 1) ★决定性判据：bag tf_static 链 + 加计均值 + 起点地面
python3 scripts/dog_platform_frame_check/frame_ground_check.py

# 2) 新旧地图逐项对比
python3 scripts/dog_platform_frame_check/map_frame_check.py

# 3) 与加计无关的几何判据（测 ground_normal_lidar 用的就是它）
python3 scripts/dog_platform_frame_check/floor_probe.py

# 4) 反证："离地高 0.125 m 那条假地面"是怎么回事
python3 scripts/dog_platform_frame_check/yband_plane_probe.py

# 5) ★改过 extrinsic_R 之后必须跑：重影（表面厚度 / 双面占比）
python3 scripts/dog_platform_frame_check/ghost_metrics.py \
    /tmp/e1/fix7/data/new_map/global.pcd /tmp/e1/R_I_sm1/data/new_map/global.pcd

# 6) 世界系 z 是否物理朝上（整图有没有上下颠倒）
python3 scripts/dog_platform_frame_check/map_orientation_check.py \
    /tmp/e1/fix7/data/new_map/global.pcd
```

## 与旧平台脚本的关系（别用错）

`scripts/pose_gravity_zdrift_check/vehicle_tilt_and_ground_check.py` 的 **[Q-A]**（从 bag IMU 解重力方向变化）
在本平台**可用**，它的 `（附）up_L 与 +y 夹角 48.45°` 正好印证本平台雷达倾斜 48°；
但 **[Q-B]**（用 +y 直方图尖峰定地面高度）写死了"雷达 +y = 上"这个 *val 平台*前提，**在狗平台上给的地面
读数（离地高 0.125 m / 法向≈+y）是假的**，见上面 `yband_plane_probe.py` 的反证。
