# pose_gravity_zdrift_check —— "z 轴持续漂移 / 世界系倾斜" 的判据与复核脚本

配套报告：`doc/pose_gravity_zdrift_fix_report.md`。
★**先读报告第 1b 节（2026-10-02 重大更正）**：该 bag 地形实测**水平（≤0.4°）**、车体从未倾斜（≤0.8°），
原先"真实 22.9° 坡道、z 变化是真地形"的结论**已作废**；真正的 z 漂移来自 **LIO 的 roll 姿态漂到 ~19°**。

| 脚本 | 用途 | 需要外参? | 依赖 |
| --- | --- | --- | --- |
| **`vehicle_tilt_and_ground_check.py`** ★推荐先跑 | **决定性判据（不需要外参）**：<br>**Q-A** 车辆物理姿态相对重力变化了多少（加计低动态 + 扣静止零偏的陀螺积分）——真走 20° 坡必然出现 ~20° 变化；<br>**Q-B** 地面在【雷达系】的坡度 / 传感器离地高（排除 NaN，用 +y 直方图尖峰定地面高度再拟合平面）。 | **否** | `rosbag2_py`、`rclpy`、`sensor_msgs`（ROS2 humble）+ numpy |
| `lidar_ground_vs_gravity.py` | **辅助判据**：地面法向 vs 加计重力。★注意它**依赖 lidar→IMU 外参**（`up_L = R_cfg^T·â0`），外参倾斜有 e° 误差时会把 e° 全读成"坡"；已修正 NaN 与"取最近平面当地面"的 bug。 | 是 | 同上 |
| `zdrift_metrics.py` | 漂移指标：回环残差 \|Δz\|/\|Δxy\|、z 跨度、行程；可选 `--pcd` 给离地高 h 与地图地面拟合。★回环残差对"整体倾斜"是**盲**的，不能单独当"无漂移"的证据。 | 否 | numpy（PCD 读取自带，`DATA binary_compressed` + LZF） |

## 用法

```bash
source /opt/ros/humble/setup.bash

# 1) ★决定性判据（不需要外参）：车辆姿态变化 + 地面坡度/离地高
python3 scripts/pose_gravity_zdrift_check/vehicle_tilt_and_ground_check.py \
    --bag /home/h/ddh/bag/930pm/bag/jieya/rosbag2_2026_10_01-00_49_15

# 2) 辅助判据（已修正 NaN/选面；只用来核对"外参倾斜 + 地面坡度"的合量）
python3 scripts/pose_gravity_zdrift_check/lidar_ground_vs_gravity.py \
    --bag /home/h/ddh/bag/930pm/bag/jieya/rosbag2_2026_10_01-00_49_15

# 3) 漂移/地图指标
python3 scripts/pose_gravity_zdrift_check/zdrift_metrics.py \
    --log ~/slam_runs/val_final/run.log --tag '方案C（5 道判据全开）' \
    --pcd ~/slam_runs/val_final/data/new_map/0.pcd
```

## 本 bag 实测结论（2026-10-02 晚复核）

```
[Q-A] 陀螺积分（扣静止零偏 0.0417 rad/s）：车辆姿态相对初值 中位 0.41°，最大 0.79°
      加计方向最大变化 9.8°，但都发生在加速瞬间（陀螺证明那刻车体只转了 ~0.4°）⇒ 运动污染
[Q-B] 地面法向 [0.005 1.000 -0.002] ⇒ 与雷达 +y 夹角 0.24~0.40°；沿行驶方向坡度 +0.09~+0.27°
      传感器离地高 恒 0.525 m（跨度 0.000 m）
[辅助] 地面 vs up_L = 11.1~11.3°（= lidar→IMU 外参/安装倾角量级，不是地形）
       旧版地面 vs up_L = 22.74°，其中 11.2° 来自"取最近平面"选到车体自身那层 11° 斜面
```

## 判读要点（避免再次误判）

- **"地面 vs 重力"不能用外参依赖的量单独判定地形**：`up_L` 里含 lidar→IMU 外参的倾斜误差；
  本平台该误差量级 **11.3°**（由"静止加计在 IMU 系与 −x 轴差 11.33°"与"地面 ⊥ 雷达 +y 0.3°"共同界定）。
- **选面规则**：地面 = "同一刚体单元内部、固定安装高度、水平"的那层 ⇒ 用**雷达 +y 直方图尖峰**定高度最稳；
  **不要**用"离传感器最近的平面"（本 bag 会选到 0.44 m 远的车体斜面，rms 7 mm 也很"干净"，极易上当）。
- **必须排除 NaN**：本 bag **20.6%** 的点是 NaN（无回波、`is_dense=false`、intensity=0），
  在含 NaN 的数组上跑 RANSAC 会生成"伪平面"（曾出现 22×22 m、厚 0.1 m、法向正好等于坐标轴的假面）。
- **回环残差对整体倾斜是盲的**：世界系/地图被刚性旋转 θ 后闭环仍闭合，但 z 会沿"虚坡"上下
  （本 bag z 跨度 1.37 m、下降速率 0.42 m/m ≈ 22.8°，而地形只有 ≤0.4° ⇒ 那部分是假漂移）。
- **`车体 rpy ≈ 0` 不能证明"地面水平"**：`vehicle_imu_mount: auto` 把初始姿态当成水平。
  但是反过来**能**用：**LIO 自报的 rpy 与"扣零偏陀螺积分"之差 = LIO 姿态误差**
  （本 bag 实测 roll 差到 **19°**，而陀螺说车体只转了 0.8° ⇒ 姿态估计有问题，z 漂移由此而来）。
- **"z 是否漂移"的三条正确判据**：① 地面在雷达系的坡度；② 离地高随时间是否恒定；
  ③ LIO 姿态与陀螺积分是否一致。三者都无异常，才能说"没有 z 漂移"。
