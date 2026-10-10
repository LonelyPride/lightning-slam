# global_map 漏加「雷达→IMU 外参」缺陷的修复确认报告

> 本文档记录 `LaserMapping::GetGlobalMap()`（建图保存主路径）**漏加 lidar→IMU 外参** 缺陷的
> 补丁内容、**与点密度无关**的公平验证指标、目视证据、端侧产物影响与复现步骤。
> 参考对象：上游原版 `/home/h/ddh/code/test/lightning-original_ws/src/lightning-lm/`
> 与本次修复后的 `src/core/lio/laser_mapping.cc`。
>
> **2026-10-01 追加**：同一根因（「雷达系点云 ↔ IMU 系位姿/坐标」混用）在**定位端**与
> **g2p5 栅格地图（`map.pgm`）**通路里也存在，已一并修复，见第 10~13 章；
> 第 14 章是同日新增的 **TF 树**修复（定位改发 `map→odom`，与车上已有 TF 树合并），
> 与上述根因无关，但因同属"定位输出边界"的问题，一并记在本报告里。
> 第 1~9 章的内容与结论不受追加影响（第 1 章的"负对照"仅针对 GetGlobalMap 那次补丁）。

---

## 1. 缺陷与补丁

### 1.1 代码位置

`src/core/lio/laser_mapping.cc:941-995` — `LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res)`

### 1.2 缺陷（改前，上游原版 `laser_mapping.cc:804-828`）

```cpp
if (use_lio_pose) {
    pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetLIOPose().matrix());
} else {
    pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetOptPose().matrix());
}
```

关键帧点云 `kf->GetCloud()` 存的是 **雷达系** 点（`MakeKF` 存入的 `scan_undistort_`），
而 `GetLIOPose()/GetOptPose()` 是 **IMU/body 系** 位姿（`vehicle_frame.h:203` 注释已写明）。
两者直接相乘等价于强行令 `T_imu_lidar = I`，漏掉本平台真实外参：

- `extrinsic_T = [0, 0, 0.004]`
- `extrinsic_R = [0,-1,0; 1,0,0; 0,0,1]`（Airy 96 竖装平台，纯 90° 航向 + 4 mm 平移）

后果（与观测一致的解释）：每个关键帧点云都以"点云自身坐标系"与"位姿坐标系"不匹配的姿态插入地图，
差的正是一个 90°（绕 z）外参；本平台雷达竖装，该错配把雷达/车体的前向与侧向按错误方式对应起来，
在车辆俯仰/横滚连续变化的路段，同一片物理地面在不同关键帧里被放到不一致的高度上，
于是叠成"多层壳"，z 向跨度异常膨胀（实测保存点云 z 跨度 25.7 m → 69.5 m）。

### 1.3 补丁（改后 `laser_mapping.cc:947-973`）

```cpp
// 个人修改：关键帧点云是【雷达系】，而关键帧位姿是【IMU 系】；拼图时必须补上 lidar→IMU 外参
const SE3 T_imu_lidar = T_imu_lidar_extrinsic();
...
if (use_lio_pose) {
    pcl::transformPointCloud(*cloud_filter, *cloud_trans,
                             vehicle_frame::ToLidarPose(kf->GetLIOPose(), T_imu_lidar).matrix());
} else {
    pcl::transformPointCloud(*cloud_filter, *cloud_trans,
                             vehicle_frame::ToLidarPose(kf->GetOptPose(), T_imu_lidar).matrix());
}
```

`vehicle_frame::ToLidarPose(T_w_body, T_imu_lidar) = T_w_body * T_imu_lidar`（`src/common/vehicle_frame.h:207`），
与以下既有约定完全一致，这正是补丁的正确性依据：

| 一致性对象 | 代码位置 |
| --- | --- |
| 前端去畸变点云上屏 | `laser_mapping.cc:329`、`laser_mapping.cc:448` |
| Pangolin 关键帧显示 | `src/ui/pangolin_window_impl.cc:67` |
| 定位端 `T_map_lidar` 约定 | `src/core/localization/lidar_loc/lidar_loc.cc` |

### 1.4 影响范围

| 产物 | 是否受补丁影响 | 证据 |
| --- | --- | --- |
| `global.pcd` | ✅ 受影响 | 修复前 228 585 点 / 修复后 67 478 点，几何差异见第 2~5 节 |
| `0.pcd`、`1.pcd`、`index.txt` | ✅ 受影响（同源切块） | 旧 `index.txt` 多一行 `1 1 0 ./data/new_map//1.pcd`（10 点）；新 `index.txt` 只有 `0.pcd`（67 477 点） |
| `map.pgm`、`map.yaml` | ❌ 不受**本补丁**影响 | 两版 md5 **完全相同**（`74ae40f3…` / `4275f698…`），它们来自另一条通路 `g2p5_->GetNewestMap()->ToROS()`（`slam.cc:176-198`） |
| 实时 SLAM 轨迹 / 前端里程计 | ❌ 不受影响 | `GetGlobalMap()` 仅被 `SaveMap()` 调用（`laser_mapping.cc:999`、`slam.cc:162`），不参与在线估计 |

> `map.pgm/map.yaml` 两版一致正好构成"补丁未扰动 2D 代价图通路"的**负对照**。
> （⚠ 2026-10-01 追加：`map.pgm` 自身有**独立的同源缺陷**（g2p5 坐标系），已一并修复，
> 修复后 `map.pgm` 会发生变化，详见第 10 章；本节的负对照结论只覆盖 GetGlobalMap 那一次补丁。）

## 2. 证据一：与独立重建点云的交叉比对（决定性判据）

用 bag 原始帧 + 日志位姿独立重建两版地图（脚本 `/tmp/gs_fix_ab.py`），再让"新保存的 `global.pcd`"
去比对两版重建图。两版重建点数**完全相同**（2 955 303 等），因此密度一致、可公平比较；
同时遍历 4 个帧对齐偏移，避免结论依赖某个偶然的帧索引约定：

| 帧对齐偏移 | 新保存 → FIX 重建（正确数学） | 新保存 → BUG 重建（漏外参） | 判定 |
| --- | --- | --- | --- |
| off=0 | 中位 **0.019 m** | 中位 0.096 m | ✅ 属于 FIX |
| off=1 | 中位 **0.017 m** | 中位 0.097 m | ✅ 属于 FIX |
| off=2 | 中位 **0.014 m** | 中位 0.098 m | ✅ 属于 FIX |
| off=3 | 中位 **0.015 m** | 中位 0.100 m | ✅ 属于 FIX |

- 判定阈值：与 FIX 重建的 NN 中位 < 0.05 m，**且** 小于与 BUG 重建中位的 1/5 → 4 个偏移全部通过。
- 反向自检（防止"中间态被误判为正确"）：旧(BUG)地图 → FIX 重建 中位 1.001~1.093 m；
  旧(BUG)地图 → 新保存 中位 1.212 m ⇒ 两版地图确有本质差异，新保存地图已完整落在"正确数学"一侧。
- z 向跨度：新保存 25.7 m、FIX 重建 27.4 m、BUG 重建 73.4~75.9 m。
- 两版重建点数逐一相等 ⇒ 差异全部来自外参，而非采样或体素差异。

（原始输出：`/tmp/gs_fix_ab.out`）

## 3. 证据二：等密度重采样后的地面平面度

`/tmp/gs_plan_check.py` 对同一批帧用**同一 0.2 m 体素**重建，两版点数完全相同（各 1 112 704 点），
此时的"局部平面度 RMS"才允许横比：

| 版本 | 点数 | z 跨度 | 局部平面度 RMS 中位 | p90 |
| --- | --- | --- | --- | --- |
| 新保存（补丁后） | 67 478 | 25.7 m | 0.029 m | 0.057 m |
| FIX 重建（正确数学） | 1 112 704 | 27.4 m | **0.017 m** | 0.045 m |
| BUG 重建（漏外参） | 1 112 704 | 75.9 m | **0.038 m** | 0.084 m |

> ⚠ "新保存"与两个重建图密度不同（6.7 万 vs 111 万点），**跨行不可直接比绝对值**；
> 严格可比的只有同密度的 `FIX 重建 0.017 m` vs `BUG 重建 0.038 m`（2.2 倍），以及 z 跨度 27.4 m vs 75.9 m。

补充：**固定半径** 0.8 m 邻域 PCA 平面度（`/tmp/gs_plan_r.py`，每版随机采同样 ~1200 个点，
同一物理体积内被同等采样）：

| 版本 | 有效采样 | 邻域平面 RMS 中位 | p90 |
| --- | --- | --- | --- |
| 新保存（补丁后） | 1199 | 0.102 m | 0.196 m |
| 旧保存（补丁前） | 1195 | 0.267 m | 0.331 m |
| FIX 重建（同密度对照） | 1199 | **0.160 m** | 0.233 m |
| BUG 重建（同密度对照） | 1198 | **0.254 m** | 0.320 m |

其中 `FIX 重建 vs BUG 重建` 一对密度严格相同（同帧、同 0.2 m 体素、同采样数、同随机种子），
平面度 0.160 m → 0.254 m（1.6 倍）。

（原始输出：`/tmp/gs_plan_check.out`、`/tmp/gs_plan_r.out`）

## 4. 证据三：地面"叠层壳"结构（完全与点密度无关）

`/tmp/gs_layers.py` 把点云按 XY 切 2×2 m 网格，格内沿 z 排序、以 >0.30 m 的间隙分层。
正确的建图里同一地面格应只有 **1 层**；漏外参时同一片地面会被多帧错位叠成多层壳：

| 版本 | 点数 | 地面格 | 每格层数中位 | 1层/2层/3层/≥4层 | 多层格占比(≥2层) |
| --- | --- | --- | --- | --- | --- |
| 新保存（FIX） | 67 478 | 73 | **1** | 43 / 17 / 8 / 5 | **41.1 %** |
| 旧保存（BUG） | 228 585 | 87 | **4** | 17 / 12 / 11 / 47 | **80.5 %** |

- 判据是"层数"而不是"点数"，所以 6.7 万点与 22.9 万点两版可以直接比。
- 该指标会被**真实竖直结构**（墙、树冠、立交上下层）混淆，因此绝对值偏高（FIX 也有 41.1 %）；
  真正有判别力的是两版的**差**：多层格占比 41.1 % vs 80.5 %、每格层数中位 1 vs 4。

（原始输出：`/tmp/gs_layers.out`）

## 5. 证据四：目视对比

`/tmp/gs_pcd_compare.png`（同视角、同裁剪/配色，由 `/tmp/gs_pcd_render.py` 生成）：

| 版本 | 点数 | x 范围 | y 范围 | z 范围 | z 跨度 |
| --- | --- | --- | --- | --- | --- |
| 新保存（补丁后） | 67 478 | [-8.3, 49.6] | [-12.4, 11.3] | [-19.5, 6.2] | **25.7 m** |
| 旧保存（补丁前） | 228 585 | [-9.8, 52.5] | [-31.4, 22.7] | [-32.1, 37.4] | **69.5 m** |

图中（左＝补丁后、右＝补丁前）：补丁后地面/路面为**单层连续面**、结构边界清晰；
补丁前整片场景沿竖直方向被"摊开"成多层壳并带拖尾（x/y 覆盖范围也明显更大）。

**运行期旁证**：UI 实时画面截图 `/tmp/gs_vis_run/ui_{3,4,5,6,7}s.win.png`
（`xwd -id <UI 窗口>` 抓取）与 `/tmp/gs_vis_run/data/new_map/global.pcd` 属于**同一次运行**
（日志同为 `global map: 67478`、`map saved`），可目视核对"实时显示（本就带外参）
↔ 保存结果（补丁后也带外参）几何一致——这正是本补丁要达成的对齐。

（原始输出：`/tmp/gs_pcd_render.out`）

## 6. 被否决 / 有局限的指标（避免后续误用）

| 指标 | 现象 | 结论 |
| --- | --- | --- |
| `gs_ground_check.py`：固定 0.5 m z 窗口的"地面厚度" | 两版都饱和在 0.500 m | 无区分度，**弃用** |
| `gs_ground2.py`：全局地面平面残差 | FIX RMS 0.499 m / BUG 2.538 m（改善 5.1 倍）；地面 z 跨度(格间) 12.48 m vs 24.90 m | 该场景含真实坡道/多层地面（FIX 侧已达 12.5 m 真实高差），**绝对值被地形污染，只能做同地形配对比较** |
| `gs_layers.py`：附带的"最底层厚度" | BUG 1.446 m 反而小于 FIX 2.680 m | 被真实竖直结构混淆，**不作为判据** |
| `gs_fix_ab.py`：附带的"局部平面度 RMS"（0.029 / 0.057 m） | 保存版与重建版密度不同 | **不可跨密度直接比**，只保留其 NN 距离结论 |
| 外置截图工具（系统截图/`ffmpeg x11grab :0`） | `run_slam_offline` 仅 ~7.4 s，抓 root 全黑 | 改用 `xwd -id <UI 窗口>` 成功 |

## 7. 端侧产物与投产建议（⚠ 重要）

定位端**不读 `global.pcd`**：`src/core/localization/lidar_loc/lidar_loc.cc:102` 调用
`map_->LoadMapIndex()`（`src/core/maps/tiled_map.cc:83` 读 `index.txt` + 分块 pcd），
分块由 `TiledMap::ConvertFromFullPCD()`（`tiled_map.cc:13`，在 `slam.cc:165-170` 被调用）生成。
因此：

- ✅ 补丁后重跑的**整目录**产物已自带正确切块结果：
  - 新：`index.txt` 只有一行分块 `0 0 0 ./data/new_map//0.pcd`（67 477 点），**无 `1.pcd`**
  - 旧：`index.txt` 多一行 `1 1 0 ./data/new_map//1.pcd`（10 点）
  - `map.pgm` / `map.yaml` 两版一致（负对照，见 1.4）
- ⚠ **不要只替换 `global.pcd`**：`global.pcd` 只是给人看的全图，端侧实际吃的是
  `index.txt + 0.pcd/1.pcd`；三者与 `global.pcd` 同源，必须**整目录一起替换**，
  否则端侧仍是旧的错误几何。
- 若手上只有旧的错误目录，理论上有两条补救路径：① 用 `TiledMap::ConvertFromFullPCD()`
  对已修正的 `global.pcd` 重切块；② 直接重新建图保存。**建议走 ②**，
  因为重切块仍会用旧 `index.txt` 里的 `start_pose` 与切块坐标约定，容易残留不一致。
- 该项（是否要为一台已部署设备重切块）需按现场情况另行决定，本报告只给出结论与风险点。

## 8. 复现步骤

```bash
# 0) 前置：补丁已编译（bin/run_slam_offline，2026-10-01 18:36 构建）
#    注意：产物写到【进程当前目录】下的 ./data/<地图名>/（slam.cc:140/149，StartSLAM("new_map")），
#          所以必须先 cd 到目标目录再启动。
cd /tmp/gs_fix_run
/home/h/ddh/code/ws/lightning_ws/src/lightning-slam/bin/run_slam_offline \
    --config /tmp/gs_fix_run/airy96_final.yaml \
    --input_bag /home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07
#    日志: /tmp/gs_fix_run/stdout.log        (含 "global map: 67478" / "map saved")
#    配置: /tmp/gs_fix_run/airy96_final.yaml
#    产物: /tmp/gs_fix_run/data/new_map/{global.pcd,0.pcd,index.txt,map.pgm,map.yaml}
#    对照: /tmp/planAplus_run/data/new_map/...   (未打补丁版，global.pcd 228585 点)

# 2) 交叉比对（独立重建 + 最近邻，决定性判据）
python3 /tmp/gs_fix_ab.py     > /tmp/gs_fix_ab.out
# 3) 等密度重采样平面度
python3 /tmp/gs_plan_check.py > /tmp/gs_plan_check.out
# 4) 固定半径邻域平面度
python3 /tmp/gs_plan_r.py     > /tmp/gs_plan_r.out
# 5) 叠层壳指标（密度无关）
python3 /tmp/gs_layers.py     > /tmp/gs_layers.out
# 6) 地面成面指标（同地形配对比较）
python3 /tmp/gs_ground2.py    > /tmp/gs_ground2.out
# 7) 目视对比图
python3 /tmp/gs_pcd_render.py > /tmp/gs_pcd_render.out   # -> /tmp/gs_pcd_compare.png
```

辅助脚本（一次性、非必须）：`/tmp/gs_vis_cap.sh`（运行期连续截屏）、`/tmp/gs_ui_cap.sh`（`xwd` 抓 UI 窗口）。

> 上述脚本已归档到仓库 `scripts/global_map_fix_check/`（与 `scripts/merge_bags.py` 同级的辅助脚本目录），
> 并已改为从自身目录导入公共模块；数据路径仍指向本次运行的 `/tmp/...` 产物。
> 用法与各指标适用范围见 `scripts/global_map_fix_check/README.md`。
> 本次归档校验：`gs_layers.py`、`gs_ground2.py`、`gs_pcd_render.py` 输出与 `/tmp` 版**逐字节一致**，
> `gs_fix_ab.py --offs 0` 复现相同结论（`0.019 m` vs `0.096 m`，z 跨度 25.7/27.4/75.9 m）。

## 9. 结论

1. **补丁已生效**（决定性）：新保存地图与"正确数学（FIX）重建"的最近邻中位距离
   0.014~0.019 m（阈值 < 0.05 m），且在 4 个帧对齐偏移下稳定成立；
   与"漏外参（BUG）重建"相差 0.096~0.100 m（5~7 倍）；反向自检（旧地图 → 两侧）中位 1.0~1.2 m，
   证明新保存地图确实落在正确一侧，而非中间态。
2. **密度无关的几何指标一致支持**：
   - 等密度重建平面度 `0.017 m → 0.038 m`（z 跨度 `27.4 m → 75.9 m`）
   - 固定半径邻域平面度 `0.160 m → 0.254 m`
   - 地面多层壳格占比 `41.1 % → 80.5 %`、每格层数中位 `1 → 4`
   - 保存点云 z 向跨度 `25.7 m → 69.5 m`
3. **目视证据**：`/tmp/gs_pcd_compare.png` 显示补丁后为单层连续地面，补丁前为多层壳/拖尾伪影；
   UI 实时截图与补丁后保存结果属同一次运行（67 478 点），可交叉核对。
4. **影响面已界定**：本补丁只改变点云类产物（`global.pcd` + `index.txt` + 分块 pcd）；
   `map.pgm/map.yaml` 两版 md5 一致，2D 代价图通路未被**本补丁**扰动（负对照）。
   （⚠ 追加：`map.pgm` 自身有独立的同源缺陷、`index.txt` 的 `start` 行随后也修正了，
   见第 10 章；替换端侧地图时请整目录一起换。）
5. **投产提醒**：端侧走 `index.txt` 分块地图，必须**整目录替换**，不能只换 `global.pcd`。

## 10. 同源缺陷的第二批修复（定位端 + g2p5 栅格地图）

### 10.1 为什么还有第二批

第一批修的是"**保存路径**"；但同一根因（**把【雷达系】点云/位姿当【IMU 系】用**）在另外两条通路上同样存在。
它们不会立刻报错（输入与输出用了同一套错坐标系，误差被"自洽"地掩盖），只会**悄悄变差**：

| # | 位置（改前） | 错在哪 | 后果 |
| --- | --- | --- | --- |
| A1 | `LaserMapping::ProjectKFs()`（`laser_mapping.cc:469`） | 用 `T_imu_cur_w * T_w_imu_kf`（**IMU 系**相对位姿）直接乘【**雷达系**】历史关键帧点云 | 投影进当前帧的"局部地图"整体歪 90°；而它正是**定位端 LidarLoc 的输入**（`localization.cpp: lio_->GetProjCloud()`） |
| A2 | `Localization` 交给 `LidarLoc`/`PGO` 的 LO/DR 状态（`localization.cpp`） | LO/DR 是 LIO 的 **IMU 系**位姿，却被当作雷达系增量与"雷达系绝对位姿"做递推：`guess_from_lo = last_abs_pose_ * delta`、`CheckLidarOdomValid`、PGO 的 DR/LO 相对因子、平滑器 `pred = pose_queue_.back()*motion`、`PoseExtrapolator::current_pose_ *= pose_incre` | 先验/外推里"横向↔垂向"互换（实测差值≈单帧位移的 **53%**），转弯时被线性放大；里程计一致性检查出现系统性偏差 |
| A3 | `slam.cc:169` 写 `index.txt` 的 `start` 功能点 | 写 `GetOptPose()`（IMU 系），而定位端把它当 **`T_map_lidar`** 初始化候选（`lidar_loc.cc: InitWithFP(input, fp.pose_)`） | 定位初始化从一个歪 90° 的位姿出发（实测整场失锁，见 10.3-4） |
| B1 | `g2p5.cc` 3 处（`ResizeMap` / `Convert3DTo2DScan` / `SetWhitePoints`） | 2.5D 投影隐含假设"z 轴向上、地面为 z=常数、极角在水平面内"，却拿到【雷达系】点云（x=左, y=上, z=前）+【IMU 系】位姿 | `map.pgm` 栅格地图整体错乱——这才是它"**两版 md5 完全相同**"的真实原因：它根本没走 `GetGlobalMap` 那条通路 |

> ★注意 B1 与 A1 的差别：**只补 `T_imu_lidar`（= 第一批补丁的写法）对 g2p5 还不够**。
> 那样得到的是【雷达系世界位姿】，点云与位姿虽然自洽，但 z 轴仍然是"前"而不是"上"，
> 3D→2D 的极角会算在"左-上"这个**竖直面**内。所以 g2p5 必须换算到 **z 轴向上**的
> 【车辆(base_link)系】：`R_vehicle_imu * extrinsic_R`。

> 【本次修复采用的坐标系约定：三个坐标系 / 两段互不代偿的外参】★前提
>
> 本工程涉及**三个彼此独立**的坐标系，**不要假设"IMU 系 = 车体系"**，也不要假设两段外参可以互相代偿：
>
> | 坐标系 | 谁生来就在这个系 | 说明 |
> | --- | --- | --- |
> | 雷达(lidar)系 | 关键帧点云 `kf->GetCloud()`、去畸变 scan、地图点云 | 单元内部坐标系（Airy 96 竖装：x=左, y=上, z=前） |
> | IMU(body)系 | ESKF 状态、`GetOptPose()/GetLIOPose()`、日志位姿 | 单元内部坐标系（IMU x=下, y=左, z=前 标称） |
> | 车辆(base_link)系 | UI/TF/栅格图的"人习惯"表达 | x=前, y=左, z=上 |
>
> | 段 | 变换 | 由谁描述（yaml） | 什么时候需要变 |
> | --- | --- | --- | --- |
> | 第 1 段 | 雷达系 → IMU 系 `T_imu_lidar` | `fasterlio.extrinsic_R/T` | **同一台雷达的单元内部固定关系**：换整车安装姿态（水平/倾斜/倒装）时**不变**；只有"换另一台雷达"或"IMU 与雷达不再是一个刚体"时才需重标 |
> | 第 2 段 | IMU 系 → 车辆系 `R_vehicle_imu` | `common.vehicle_imu_mount`(auto/identity/x_down/x_up) 或 `common.vehicle_imu_R` | 换安装姿态即变；`auto` 时**运行期由静止重力求解**，可描述任意倾斜 |
>
> 由此可以看出本次 4 处改动的"最小必要换算"：
> A1/A2/A3 **只需要第 1 段**（它们上下游都在雷达系：定位输入是雷达系点云、地图是雷达系拼的、
> `index.txt` 的 `start` 语义是 `T_map_lidar`），因此**完全不涉及车辆系、也不需要知道 IMU 怎么装在车上**；
> 只有 **B1(g2p5) 需要第 2 段**，因为它的 3D→2D 算法硬性要求"z 朝上"。
>
> 本平台运行期实例（日志 `laser_mapping.cc:235`，auto 模式）：
>
> ```
> R_vehicle_imu =
>   -0.193947 -0.00957521    0.980965      ← 列 = IMU 各轴在车辆系中的方向：
>  -0.0493103    0.998784           0        第1列≈(0,0,-1)=下；第2列≈(0,1,0)=左；
>   -0.979772  -0.0483717   -0.194183        第3列≈(1,0,0) 但向下偏 11.25°
> ```
>
> 该矩阵**不是干净的 90°**：约 11.25° 的偏差正是"标称竖装"与实车重力方向的实测差
> （见 `doc/airy96_config_guide.md` 第 11 节）。这也说明：代码既没有把"IMU=车体"当成前提，
> 也没有把 90° 写死，而是把两段外参**各自独立地**管起来、第 2 段运行时现取（`g2p5.cc` 不缓存）。

### 10.2 代码改动（逐条）

| # | 文件 | 改动 |
| --- | --- | --- |
| A1 | `src/core/lio/laser_mapping.cc:474-487` | `SE3 pose = T_imu_lidar⁻¹ * pose_cur * kf->GetLIOPose() * T_imu_lidar;`（左右各补一次外参） |
| A2 | `src/core/localization/localization.{h,cpp}` | 新增 `Localization::ToLidarFrameState()`，只在两个入口（`ProcessIMUMsg` 的 DR、`LidarOdomProcCloud` 的 LO）把 `T_w_imu` 换成 `T_w_lidar`，之后 lidar_loc / PGO / 平滑器 / 外推器**内部统一在雷达系**工作 |
| A3 | `src/core/system/slam.cc:173-179` | `start_pose = ToLidarPose(GetAllKeyframes().front()->GetOptPose(), T_imu_lidar)`；为此把 `LaserMapping::T_imu_lidar_extrinsic()` 由 private 移为 public（`laser_mapping.h:129`） |
| B1 | `src/core/g2p5/g2p5.{h,cc}` | 新增工作坐标系换算：`ToWorkingCloud()`（雷达系点云→车辆系）、`ToWorkingPose()`（IMU 系位姿→车辆系）、`R_vehicle_lidar()`；`Init()` 读入 `fasterlio.extrinsic_R/T`；`DetectPlaneCoeffs()` 改为接收已换算点云；三处调用点统一改用换算后的点云/位姿 |

编译：`colcon build --packages-select lightning` 通过，`bin/*` 于 2026-10-01 19:19-19:20 重建，
本次改动未引入新的编译告警（仅命中 `-Wsign-compare` 等既有告警）。

### 10.3 证据（全部可复现，脚本见 `scripts/global_map_fix_check/`）

**(1) 回归：本次改动不扰动建图结果（决定性）**
同一 bag、同一配置，三次重跑（旧代码 18:37 参考 / 新代码 cfg 0+0 / 新代码 cfg 1.81+(-1.81)）：

- `global.pcd`：均 **67478 点**，三者逐点 x/y/z **最大差 = 0.000e+00 m（完全相同）**；
- `0.pcd`（端侧实际读的分块）：均 **67477 点**；
- 首个关键帧状态 `LIO: create kf 0, state: -0.000566496 0.000271783 0.000909619` 三次一致。

> 注：PCD 文件字节不同（md5/体积有差异），原因是点结构 `PointXYZIT` 里存在**未初始化的对齐填充字节**，
> 压缩后字节数会随机波动；几何内容逐点完全相同 ⇒ **本工程产物校验不要只看 md5，要比较点坐标。**

**(2) g2p5（`map.pgm`）与"已复核点云"的一致性**（同一次运行的产物同源、同世界系，可直接比较）

| 指标（与密度无关） | 旧代码 cfg 0/0 | 新代码 cfg 0/0 | 新代码 cfg 1.81/-1.81 |
| --- | --- | --- | --- |
| 栅格范围超出点云 XY 包围盒（x / y） | 1.7 / 2.7，**20.0 / 12.3 m** | 1.0 / 4.5，**0.9 / 1.8 m** | 1.0 / 4.5，**0.9 / 1.8 m** |
| 占用格 → 最近点云点 < 0.3 m 的占比 | 88.7%（p90 0.362 m） | 90.2%（p90 0.296 m） | **97.9%（p90 0.044 m）** |
| 障碍点（离地 0.3~1.8 m）→ 占用格 < 0.3 m | **41.6%**（中位 0.660 m） | 56.7%（中位 0.227 m） | **93.5%**（中位 0.042 m） |
| 占用格 / 空闲格 数量 | 11361 / 5648 | 5113 / 25062 | 14736 / 13739 |
| 栅格尺寸 | 56.8 × 56.0 m | 54.4 × 26.4 m | 54.4 × 26.4 m |

- 判据 1：g2p5 的 `ResizeMap` 只按"点云 XY 外扩 1 m"扩图，栅格范围应 ≈ 点云包围盒 +1 m。
  旧图在 y 方向超出点云 **20.0 / 12.3 m**（那里根本没有点），新图收敛到 0.9 / 1.8 m
  ⇒ 旧栅格把几何铺到了不存在结构的地方。
- 判据 2（完备性）：真实障碍点里能被栅格图标成占用格的占比 **41.6% → 93.5%**、中位距离 **0.660 m → 0.042 m**
  ⇒ 新 `map.pgm` 真正画出了走廊两侧的墙；旧图基本"没画对"。

**(3) 定位输入端 A1（`ProjectKFs`）的量化**（`gs_projkfs_check.py`）
20 个样本（历史帧取 0.8 s / 平均 0.94 m 之前），把投影点云与**当前帧实测点云**
（两者同在当前帧雷达系下，静态场景应重合）比较最近邻：

| 投影方式 | NN 中位 | 均值 | max |
| --- | --- | --- | --- |
| FIX（补丁后） | **0.153 m** | 0.196 m | 0.546 m |
| BUG（补丁前） | **1.074 m** | 1.167 m | 2.481 m |

⇒ 补丁前喂给定位的"局部地图"整体歪 90°、平均错开 1.07 m（7 倍），补丁后重新贴合到 0.15 m。

**(4) 定位端 A3（`index.txt` 的 `start` 功能点）A/B**（`gs_loc_check.py`）
用**同一份修复后地图**、同一条 bag 跑 `run_loc_offline`，只把 `index.txt` 的 `start` 行换回旧值：

| | L1：修复后（start = 雷达系位姿） | L2：旧值（start = IMU 系位姿） |
| --- | --- | --- |
| FP 初始化 | `init with fp: start`，score **2.170**（阈值 1.8） | score **0.732**，仍被判为"已初始化"，位姿 (0.289, −0.085, 0.167) |
| 36 帧匹配分值 | 中位 **2.228**（min 1.971 / max 2.403），**100% > 阈值** | 中位 **0.499**（min 0.015 / max 0.866），**0% > 阈值** |
| 定位位姿 vs LIO 轨迹（同世界系，可直接比） | XY 中位 **0.026 m**，p90 0.057，max 0.096 m | XY 中位 **1.445 m**，p90 3.285，max 3.416 m |
| 是否跟随车辆运动 | 轨迹范围与 LIO 完全一致（x[−3.3, 0.2] y[0, 3.8]） | 未跟上（y[−0.6, 0.2]，横向漂出去） |

⇒ `start` 功能点坐标系错 90° 会让**整场定位失锁**（分值全废、位置差 1.4~3.4 m）。

**(5) 定位端 A2（先验/外推坐标系）量级**（`gs_loc_prior_check.py`，307 帧真实位姿序列）

- 单帧平均位移 0.039 m（本数据集车辆只走了约 4 m 的缓行段）；
- 补丁前后的先验位姿差：平移 中位 **0.021 m** / p95 0.039 / max 0.061 m；姿态 中位 0.53° / max 6.54°；
- **平移差 ≈ 单帧位移的 52.9%**（横向分量被 90° 外参换到垂向，前向分量不受影响）。

⇒ 属于**结构性错误**：误差随单帧位移线性放大（例如 5 m/s、0.5 s 预测间隔、1 m 横向运动时先验可偏 ~1.4 m）；
本数据集缓行故绝对值仅厘米级，动态场景收益需另用"有转弯/中高速"的 bag 复测。

**(6) 目视证据**
`/tmp/gs_g2p5_old.png` / `/tmp/gs_g2p5_new.png`（左：`map.pgm` + 轨迹；右：同次运行 `global.pcd` 俯视按 z 着色）：
新图在走廊两侧画出连续黑墙、内部为白（可通行），与右侧点云结构逐段对应；
旧图只有起点一团黑斑 + 若干斜向拖尾，其余全是"未知"灰。

### 10.4 配置语义变化（★需同步 yaml）

g2p5 换算到车辆系后，`g2p5.lidar_height / floor_height` **回归标准含义**（地面在传感器下方 h 处）：

```yaml
g2p5:
  esti_floor: false        # 本数据有坡道，动态拟合会因"地面不水平"被判失败（values[2]<0.99），建议写死
  lidar_height: 1.81       # ★换算后工作坐标系原点离地高度 h；该原点 = IMU 原点（换算只换朝向不搬原点），
                           #   本平台 IMU 在雷达内 ⇒ 实测雷达离地高度即可（见 gs_ground_height.py）；
                           #   IMU 与雷达分离时须改按 IMU 高度填
  floor_height: -1.81      # ★= -h，地面在换算后点云坐标系中的 z
  min_th_floor: 0.2        # 高于地面该高度的障碍写入栅格
  max_th_floor: 2.0
```

> ⚠ 两段外参各行其道（见 10.2 前的坐标系约定）：`fasterlio.extrinsic_R/T` = 雷达↔IMU 的**单元内部**
> 关系；`common.vehicle_imu_*` = IMU↔车体的**安装姿态**。改一段去凑另一段会把"标定值"污染成"补偿值"，
> 下次换安装姿态就失效。

实测依据（`gs_ground_height.py`，120 个采样位置）：雷达离地高度 h 中位 **1.812 m**（5%~95%: 1.622~2.163 m）；
世界系下局部地面法向 z 分量中位 0.9665（有坡道/匝道，故不宜用 0.99 的"地面水平"判据做动态估计）。

> 之前沿用 lidar_slam Airy 平台的 `0.0 / 0.0` 是在"**点云坐标系搞错**"的前提下配的：
> 那时雷达系 z = 前向，"地面 z≈0"根本不是地面。若暂时不改配置（保持 0.0/0.0），
> 修复后仍比旧代码好（完备性 41.6% → 56.7%），但地图切层位置偏高、覆盖率明显不如 1.81 配置。

### 10.5 复现步骤

```bash
# 0) 编译（本次改动）
cd /home/h/ddh/code/ws/lightning_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select lightning          # -> bin/ 重新生成

# 1) 两次建图 + 两次定位（脚本内含 A/B 配置与"把 index.txt 换回旧 start"的处理）
bash /home/h/ddh/code/ws/lightning_ws/src/lightning-slam/scripts/global_map_fix_check/run_g2p5_loc_checks.sh
#    产物：/tmp/g2p5fix_run（新代码 cfg 1.81）、/tmp/g2p5fix_run_cfg0（新代码 cfg 0/0）、
#          /tmp/loc_fixrun（L1）、/tmp/loc_oldstart_run（L2）
#    对照：/tmp/gs_fix_run（第一批修复时的运行目录，勿动）

# 2) 指标
S=/home/h/ddh/code/ws/lightning_ws/src/lightning-slam/scripts/global_map_fix_check
python3 $S/gs_g2p5_check.py --run /tmp/gs_fix_run        --tag 'old-code cfg 0/0'  --render /tmp/gs_g2p5_old.png
python3 $S/gs_g2p5_check.py --run /tmp/g2p5fix_run_cfg0 --tag 'new-code cfg 0/0'  --render /tmp/gs_g2p5_cfg0.png
python3 $S/gs_g2p5_check.py --run /tmp/g2p5fix_run      --tag 'new-code cfg 1.81' --render /tmp/gs_g2p5_new.png
python3 $S/gs_loc_check.py  --loc-log /tmp/loc_fixrun/stdout.log       --tag 'L1 fixed index.txt'
python3 $S/gs_loc_check.py  --loc-log /tmp/loc_oldstart_run/stdout.log --tag 'L2 old index.txt'
python3 $S/gs_projkfs_check.py     # A1：投影点云 vs 当前帧实测点云
python3 $S/gs_loc_prior_check.py   # A2：先验坐标系差量级
python3 $S/gs_ground_height.py     # 雷达离地高度/地面水平性实测
```

> 所有 `run_*` 程序在退出时都会命中 Pangolin/GLX 断言（`EXIT=134`），这是**本机离线跑 UI 的既有现象**
> （对照 `/tmp/gs_fix_run` 同样如此），产物在此之前已全部落盘（日志里可见 `map saved` / `dynamic maps saved`）。

### 10.6 结论补充与遗留项

1. **定位的"核心"不需要改**：定位是"雷达系扫描 ↔ 雷达系拼出的地图"之间的直接配准，
   补丁后地图与点云同系、`T_map_lidar` 语义自洽，**不需要**在配准里再补外参。
   要改的是**跨坐标系的三处**：A1 输入局部地图、A2 相对运动先验、A3 初始化功能点。
2. **g2p5 必须改**，且比 A1~A3 更"重"：要引入"车辆(base_link)系"这一层换算（见 10.2-B1），
   并同步 `lidar_height/floor_height` 配置（见 10.4）。
3. 端侧替换提醒（在第 9 章第 5 条基础上的补充）：现在 `index.txt` 里的 `start` 行语义也变了
   （IMU 系 → 雷达系），因此**地图目录必须整目录一起替换**；只换 `global.pcd` 或混用新旧 `index.txt`
   会造成"地图对了、初始化位姿歪 90°"的组合故障（L2 已复现该故障形态）。
4. 遗留项（本次**未改**，供后续决策）：
   - `LidarLoc::InitWithFP()`（`lidar_loc.cc:297`）只用"NDT 是否收敛"判初始化成功，**没有比分门槛**，
     因此从歪 90° 的位姿也能"初始化成功"（L2 实测 score 0.732 < 阈值 1.8）。
     建议后续加 `if (fitness_score < options_.min_init_confidence_) return false;`。
     **→ 已在第 11 章落地**（限定在初始化阶段），并查明在线入口还会额外注入单位阵初值，
     两者共同造成"在线定位完全不可用"。
   - `YawSearch()` 已被注释掉（`lidar_loc.cc:296`），初始化失败时不会再做航向搜索。
   - 本数据集车辆只走了约 4 m（缓行）：A1/A2 在高动态场景下的收益需另找一段有转弯、中高速的 bag 复测。

## 11. 第三批：**在线定位入口**的初值坐标系与初始化门槛（"同一 bag 建图后定位完全不可用"的根因）

### 11.1 现象与用户原始链路

复现路径（**用同一条 bag 建图、再用同一条 bag 定位**）：

```bash
cd /home/h/ddh/code/ws/lightning_ws                       # 地图写到/读自 ./data/（相对【进程启动目录】）
ros2 run lightning run_slam_online  --config src/lightning-slam/config/default_airy96.yaml   # 1) 建图
ros2 run lightning run_loc_online   --config src/lightning-slam/config/default_airy96.yaml   # 2) 定位
# bag: /home/h/ddh/bag/follow_car/20260914/decompressed/rosbag2_2026_09_15-10_22_04  (238 s, 6.1 GiB)
# 地图: ./data/new_map/  (map.pgm 116.8 x 25.6 m，2 个分块，功能点 start)
```

现象：**定位完全不可用** —— 初始化分值 0.679（阈值 1.8），随后分值塌到 0.07~0.14、位姿乱跳；
而**同一份地图**用**离线**入口 `run_loc_offline` 却正常。用户怀疑"还是坐标转换的问题"——判断正确。

> 附注（两份 bag 的等价性）：`decompressed/rosbag2_2026_09_15-10_22_04`（raw PointCloud2，6.1 GiB）与
> `三楼走廊-压缩激光-IMU-里程计-静态TF/rosbag2_2026_09_14-04_06_39`（CompressedPointCloud2，819 MiB）
> 是**同一次行驶的同一份数据**：消息头 `header.stamp` 完全相同（起始 1789358800.x），
> 只是"解压重录"那一次的 rosbag **记录时间**落在 9/15 10:22（`ros2 bag info` 显示的 Start 是记录时间，
> 不是消息头时间戳）。所以"建图用哪个包"不影响结论；本文复现用的是 6.1 GiB 那个。

### 11.2 根因：两个**在线入口独有**的缺陷，任一都足以让定位不可用

| # | 位置（改前） | 错在哪 | 后果 |
| --- | --- | --- | --- |
| D1 | `src/app/run_loc_online.cc`（`main`） | 无条件 `loc.SetInitPose(SE3())`：把**单位阵**当成 `T_map_lidar` 注入 | `lidar_loc` 内部用的位姿是**雷达系在地图下**的 `T_map_lidar`；它与"雷达系与地图原点重合"之间还差 **lidar→IMU 外参**与 **IMU→车辆安装角**两级外参（本例 lidar→IMU 外参本身就不是单位阵，见日志 `localization.cpp:67`）。单位阵 = "雷达在原点、姿态与地图对齐" → NDT 收敛到错误极小值（实测 0.663 / 0.679） |
| D2 | `src/core/localization/lidar_loc/lidar_loc.cc`（`Localize()` 的初始化分支） | `min_init_confidence_` 的分值把关被写成 `if/else` **两个分支都置 `loc_success = true`**（恒真），阈值形同虚设 | D1 的错误初值被**无条件接受** → `InitWithFP` 把 `loc_inited_` 置 true → ① 功能点回退分支（`start`，分值通常 >2.0）**永远不会被执行**；② 重试逻辑全部在 `!loc_inited_` 分支内，**再没有重新初始化的机会** → 带着错误初值一路漂移 = "完全不可用" |

**为什么离线一直正常**：`run_loc_offline.cc` 从不调用 `SetInitPose()`，直接用 `index.txt` 的功能点 `start`
（`lidar_loc.cc:511-520` 逐个候选尝试）；而 D2 的门槛只有在"存在错误候选"时才有意义。
这就解释了用户观察到的"**离线可用、在线不可用**"。

> D2 正是第 10.6 节遗留项 #1 的实锤（原文："只用 NDT 是否收敛判初始化成功，没有比分门槛…
> 建议后续加 `if (fitness_score < options_.min_init_confidence_) return false;`"）。
> 本批修复把这一条**限定在初始化阶段**落地（跟踪阶段不受阈值约束）。

### 11.3 代码改动

| # | 文件 | 改动 |
| --- | --- | --- |
| D1 | `src/app/run_loc_online.cc` | 删掉无条件的 `SetInitPose(SE3())`，默认改调新增的 `LocSystem::Start()`（**不注入外部位姿** → 与离线入口一致，由功能点 `start` 初始化）；新增**可选** `--init_pose "x,y,z,qx,qy,qz,qw"`（语义 = **T_map_lidar**）+ `ParseInitPose()`（个数/四元数范数校验，解析失败自动退回 `Start()`） |
| D1 | `src/core/system/loc_system.{h,cc}` | 新增 `LocSystem::Start()`（只置 `loc_started_ = true` 并打印 `start loc without external init pose ...`）；`SetInitPose()` 的文档注释写明"必须是 `T_map_lidar`，不要传车辆/IMU 位姿；车停在建图起点时请改用 `Start()`" |
| D2 | `src/core/localization/lidar_loc/lidar_loc.cc` | 初始化分支加门槛：`if (loc_inited_ == false) { loc_success = (confidence > options_.min_init_confidence_); if (!loc_success) LOG(WARNING) << "init confidence too low: X <= 1.8, reject this init candidate"; }`；跟踪阶段（`loc_inited_ == true`）保持原逻辑不变 |

编译：`colcon build --packages-select lightning --cmake-args -DCMAKE_BUILD_TYPE=Release`，
**2026-10-01 20:40** 重建（`bin/` 与 `install/` 同步；构建日志 `/tmp/loc_fix_build.log`）。
★用户 20:28 那次失败用的仍是**旧二进制**（`ros2 run` 走 `install/`），必须以本次重建后的为准。

### 11.4 证据

**(1) 修复前（用户 20:28 原始日志：同一条 bag + 同一份地图）**

```
loc_system.cc:75]  set init pose: 0 0 0, 0 0 0 1           ← D1：单位阵被当作 T_map_lidar
lidar_loc.cc:312]  fitness_score is: 0.678654, global_pose is: 0 0 0
lidar_loc.cc:487]  init with external pose: 0 0 0          ← D2：0.679 << 1.8 却被接受
confidence: 0.678654 / 0.707905 / 0.574389 / 0.737247 / 0.131521 / 0.102785 / 0.130585 / 0.142441 / 0.0704175 …
```

同现象在复现环境重现：`fitness_score is: 0.662846, global_pose is: 0 0 0` → `init with external pose: 0 0 0`，
19 帧里 95 % 的分值 < 1.8（均值 0.817），位姿在地图原点附近来回晃。

**(2) 修复后（用户原始命令 + 原始 bag + 原始地图，全程 238 s）**

```
loc_system.cc:87]  start loc without external init pose, use map functional point 'start' instead
lidar_loc.cc:312]  fitness_score is: 2.18905, global_pose is: 0.00398793 0.000906416 3.28313e-05
lidar_loc.cc:313]   [Loc init pose]:  0.110278 -0.00265107 -0.00348959
lidar_loc.cc:516]  init with fp: start                       ← 用地图功能点 start 初始化成功
```

`gs_loc_check.py --loc-log /tmp/loc_bagB_full.log --ref-log /tmp/run_slam_online.INFO`：

| 指标 | 修复前 | 修复后（全程，186 帧定位输出） |
| --- | --- | --- |
| 初始化分值 | 0.679（被误接受） | **2.189**（> 1.8 阈值） |
| 定位分值 中位 / >1.8 占比 | 0.13~0.74 / ≈5 % | **2.056 / 96.2 %**（min 1.559、max 2.327） |
| 定位位姿 ↔ 建图 LIO 轨迹（XY 最近邻） | 失锁（不可比） | **中位 0.040 m，p90 0.070 m，max 0.138 m** |
| 轨迹覆盖 | 地图原点附近乱跳 | x=[-3.5, 86.7] y=[-1.0, 3.3]，与建图轨迹范围**完全一致** |

**(3) 单元级 A/B：门槛真的在起作用，且能自动恢复**

(a) 在**用户的原始数据 + 原始地图**上（脚本 `run_loc_online_checks.sh` 自动跑这两组）：

| 组 | 注入的初值 | 日志 | 结果 |
| --- | --- | --- | --- |
| A | 无（默认） | `loc_system.cc:87 start loc without external init pose` → `lidar_loc.cc:516 init with fp: start` | 13 帧均值 **2.160**，0 帧低于阈值 |
| B | 单位阵 `--init_pose 0,0,0,0,0,0,1` | `run_loc_online.cc:77 use given init pose (T_map_lidar): 0 0 0, 0 0 0 1` → `lidar_loc.cc:865 init confidence too low: 0.678654 <= 1.8, reject` → `lidar_loc.cc:516 init with fp: start` | 14 帧均值 **2.054**，**初始化成功** |

★B 组被拒的分值 **0.678654 与用户 20:28 失败日志里的 `fitness_score is: 0.678654` 完全相同** ——
也就是说用户那次失败用的正是这个单位阵初值，修复后同一初值被门槛拦住并自动回退 `start`。

(b) 另一组同源数据（bag `/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07` + 新代码地图）：

```
run_loc_online.cc:77]  use given init pose (T_map_lidar): 0 0 0, 0 0 0 1
lidar_loc.cc:865]      init confidence too low: 0.625237 <= 1.8, reject this init candidate   ← 被拒绝
lidar_loc.cc:312]      fitness_score is: 2.17009 …
lidar_loc.cc:516]      init with fp: start                                                    ← 自动回退并成功
```

结论：**修掉 D1（不再注入单位阵）并不等于可以不要 D2** —— 只要将来又有人/有代码给出错误初值，
门槛就是最后一道防线；两者同时修掉，才做到"错误初值不再致命"。

### 11.5 复现步骤（脚本：`scripts/global_map_fix_check/run_loc_online_checks.sh`）

```bash
S=scripts/global_map_fix_check
# A) 用户原始链路（默认参数即"三楼走廊 bag + 用户地图 + 默认 config"）
bash $S/run_loc_online_checks.sh
#    该脚本会自动再跑一遍 B) 故意注入单位阵初值，用于验证门槛是否真的在拦
# 指标（在线日志同样适用）
python3 $S/gs_loc_check.py --loc-log /tmp/loc_online_check_A.log \
        --ref-log /tmp/run_slam_online.INFO --tag 'online-A'
# 初值/门槛的关键日志行
grep -E 'start loc without|use given init pose|init confidence too low|init with (fp|external)|fitness_score' \
     /tmp/loc_online_check_A.log
```

### 11.6 结论与投产提醒

1. **在线定位的根因确实是"坐标系"**：在线入口把**单位阵**当成了 `T_map_lidar`（D1）；
   而它之所以能毁掉整场定位，是因为**初始化门槛失效**（D2）。两者都只在**在线**入口出现，
   所以呈现为"离线好、在线废"。
2. **必须用重建后的二进制**：`install/` 与 `bin/` 已于 2026-10-01 20:40 同步重建。
   用户此前那条 `ros2 run lightning run_loc_online …` 的失败日志来自旧二进制。
3. **定位请从地图起点附近开始**：默认初值 = 地图 `index.txt` 的功能点 `start`（= 建图首帧的 `T_map_lidar`）。
   若定位时车不在建图起点，请显式给 `--init_pose "x,y,z,qx,qy,qz,qw"`（**T_map_lidar**，含 lidar→IMU 外参）；
   否则候选 `start` 会因分值不足被拒（这是**正确**行为），随后依次尝试其它候选。
4. **`./data/recover_pose.txt` 机制（本次查明）**：yaml 里没有这个键，路径硬编码在 `lidar_loc.h:65`
   （相对【进程启动目录】）；**每帧**定位结果都会写入（`lidar_loc.cc:782`），启动时若文件存在，
   它会被当作名为 `recover` 的功能点加入初始化候选（`lidar_loc.cc:112`，排在 `start` 之后）。
   上一次跑飞后它保存的就是错误位姿——修复前"门槛失效"会让它被无条件接受，
   这是"重启一次、失败一次"的自我放大来源。现在它只是受门槛约束的普通候选；
   **不要手工填写/复制该文件**，如需清空直接删除（下次运行会重新生成）。
5. **地图不需要重建**：本次验证用的就是用户 20:25 建的那份地图（`map.pgm` + 2 个分块 + `index.txt`），
   修复后定位精度 4~14 cm，说明该地图与 bag 同源、可用。修复落在**定位侧**，无需重新建图
   （但替换端侧地图时请遵守第 9/10 章的"**整目录替换**"原则）。

---

## 12. 手动初始位姿 / 随时重新定位（RViz "2D Pose Estimate" → `/initialpose`）

> 本章为**新功能**（不是缺陷修复），2026-10-01 与第 11 章同一轮改动；涉及文件：
> `src/core/system/loc_system.h/.cc`、`src/core/localization/localization.h/.cpp`、
> `src/core/localization/lidar_loc/lidar_loc.h/.cc`、`src/io/yaml_io.h/.cc`、
> `src/app/run_loc_online.cc`、`config/default_airy96.yaml`，新增
> `scripts/global_map_fix_check/send_initial_pose.sh`。

### 12.1 需求

> "在定位时，我希望可以通过手动发送话题的形式给定初始位姿，比如通过 RViz 上的按钮
> 手动发一个初始位姿，定位模块接收到后从该点处进行定位，这样我可以随时校准车的初始位置。"

实现结果：`run_loc_online` 现在订阅 **`/initialpose`**（`geometry_msgs/msg/PoseWithCovarianceStamped`，
即 RViz 工具栏 **"2D Pose Estimate"** 的默认话题），**运行中任意时刻**收到该消息就
**以该位姿为种子重新初始化**（`loc_inited_` 清零 → 下一帧点云重新配准），因此可以随时把定位
"拽回"车真正所在的位置；不必重启进程，也不影响启动时的默认初始化（默认仍是地图功能点 `start`）。

### 12.2 数据流

| 环节 | 代码位置 | 行为 |
| --- | --- | --- |
| 订阅话题 | `loc_system.cc` `Init()` | 话题名 `common.initial_pose_topic`（缺省 `/initialpose`，空串可关闭）；QoS 深度 1（只关心最后一次点击） |
| 语义判定 | 同上 | `common.initial_pose_frame`：`vehicle`（缺省）= 车辆 base_link 系；`lidar` = 雷达系 `T_map_lidar` |
| 回调 | `LocSystem::OnInitialPose()` | 四元数归一化 + 合法性校验；`frame_id != "map"` 时告警（不拒绝）；打印收到的位姿 |
| 坐标系换算 | `Localization::SetExternalVehiclePose()` | `T_map_lidar = ToLidarPose(ToBodyPose(T_map_vehicle), T_imu_lidar_)`，即先撤销 IMU→车辆安装旋转 `M`，再乘"雷达→IMU"外参；两个位姿都打进日志 |
| 下发重定位 | `Localization::SetExternalPose()` → `LidarLoc::SetInitialPose()` | 置 `initial_pose_set_ = true`、`loc_inited_ = false`（**这就是"重新初始化"的开关**） |
| 实际重定位 | `LidarLoc::Align()` 的 `!loc_inited_` 分支 | `map_->LoadOnPose()` → `InitWithFP(input, seed, /*wide_seed_search=*/true)`；**手动种子排在功能点候选之前** |
| 门槛 | `LidarLoc::Localize()` | 种子仍须 `score > min_init_confidence_`（1.8），否则打印 `init confidence too low ... reject this init candidate` 并继续尝试其它候选（不会带飞定位） |

### 12.3 坐标系语义（本平台最容易踩的坑）

- RViz 的 "2D Pose Estimate" 给的是 **Fixed Frame 下 `base_link` 的位姿**，所以本工程要求
  **RViz 的 Fixed Frame 设为 `map`**（本工程地图帧名，见 `localization_result.cc` 发布的 `map→base_link`）。
- 定位模块内部（`lidar_loc` / PGO / 地图 / 输入点云）统一使用**雷达系位姿 `T_map_lidar`**；
  两者相差两级外参（本平台雷达/IMU 竖装 ⇒ 姿态差约 90°、位置差约 4 mm）。
  **若把车辆位姿直接当 `T_map_lidar` 用，会被门槛拒掉**（第 11 章 D1 就是这个错误的"单位阵版本"）。
  实测（失败对照）：把车辆位姿按雷达系解释时最终分值 1.06~1.15；
  正确换算后同一时刻的分值 1.91~2.05。
- 换算只用现成的 `common/vehicle_frame.h` 工具，与 map→base_link 的 TF 发布、UI 显示走同一条链路。

### 12.4 顺带发现并修掉的两个缺陷（都在"手动重定位"这条路上）

**D3：手动位姿分支漏了地图块加载（`lidar_loc.cc`）**

功能点循环每试一个候选都会先 `map_->LoadOnPose(fp.pose_)`，而手动位姿分支没有 ⇒ NDT 目标
仍是"上一处加载的地图块"。在 x≈60 m 处手动拉回时表现为：**同一路段的配准分值被压低**
（1× 速无干预参照 median **2.065**、低于 1.8 占比 0 %；手动拉回后 median **1.726**、**66.7 %**），
既可能被门槛误拒，即使成功也会让后续跟踪分值偏低。
**修复**：手动分支同样先 `LoadOnPose(initial_pose_)`。地图块的 NDT 目标由地图线程异步更新，
若本帧尚未更新，下一帧会带着新目标重试 —— `initial_pose_set_` 在成功前一直为 `true`。

**D4：手动种子的"点击延迟"与"附近局部最优"（`lidar_loc.cc`）**

手动点选天生带偏差：① 用户在"他看到的现在"点下，而正在配准的那一帧点云还在后面
⇒ 车速 1 m/s 时种子偏 0.5~2 m；② 拖出的车头方向也可能差十几度。而 NDT 单次配准只有
**4 次迭代、收敛半径小**，实测种子偏 ~1.3 m 时第一轮分值只有 **1.4~1.78 < 1.8**（被拒）。
即使侥幸接受（1.875），也只是收敛到"真值附近的局部最优"，该偏差会被后续跟踪继承
（预测初值总在偏差点附近 ⇒ 每帧都收敛回偏差点，分值系统性偏低）。
**修复**（仅对手动种子，且可用 yaml 关掉）：

1. 第一轮细配准被拒时，先用**粗分辨率(5 m) NDT + 小范围 yaw 搜索**（`±30°`、7 个采样，含原种子）
   重新找种子（粗分辨率代价函数更平滑、收敛半径更大），再做细配准；
2. 接受后**再迭代精修 2 次**（只在分值变好时采用，绝不劣化），把手动初值真正落到
   与正常跟踪一致的最优解上。实测：`1.90959 → 1.92345 → 2.04647`，位姿从种子
   **39.198 m 收敛到 39.880 m**（正是 0.68 m 的点击延迟偏差），与同路段正常跟踪分值（1.97~2.00）一致；
3. 两个门槛语义**没有放宽**：最终仍由同一个 `min_init_confidence_`（1.8）把关。

### 12.5 使用方法

**A. RViz（推荐）**
```bash
cd ~/ddh/code/ws/lightning_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lightning run_loc_online --config src/lightning-slam/config/default_airy96.yaml
# 另开 RViz2：Fixed Frame 填 map，加入 TF/点云显示；工具栏点 "2D Pose Estimate"
#  → 在地图上【车当前所在位置】按下并拖出车头方向。日志会出现：
#    [manual init] T_map_vehicle(base_link): ...
#    [manual init] -> T_map_lidar (...): ...
#    Set initial pose is: ...      → init with external pose: ...
```

**B. 命令行 / 脚本**（新增 `scripts/global_map_fix_check/send_initial_pose.sh`）
```bash
D=scripts/global_map_fix_check
bash $D/send_initial_pose.sh --x 12.30 --y -0.40 --yaw 0                 # 位置 + 车头朝向(度)
bash $D/send_initial_pose.sh --x 12.30 --y -0.40 --z 0 --quat 0,0,0.7071,0.7071
bash $D/send_initial_pose.sh --from-tf                                   # 回灌定位当前上报的位姿
```

### 12.6 实测验证（数据 = 用户自己的 bag + 用户地图，0.5 倍速播放）

| 用例 | 场景 | 结果 |
| --- | --- | --- |
| A | 运行中故意点错（位置正确、车头 **+90°**） | 被拒 `1.10/1.06/1.74 <= 1.8, reject`；粗搜索执行后明确报 `rough seed search did not help`；随后回退功能点 `start` ⇒ **错误点击不会带飞定位** |
| B（修复前） | 运行中"正确点击" | 被拒 `1.4~1.78 < 1.8`（**本用例暴露了 D4，促成 12.4 的修复**） |
| C（修复后） | 运行中"正确点击"（种子取自定位自己发布的 `map→base_link`，天然滞后 ~0.5 m） | 接受 `1.90959` → 精修 `1.92345` → **`2.04647`**，位姿 **+0.68 m** 收敛到真值 |
| C-后续 | 拉回之后 15 帧的跟踪质量 | 分值 median **2.106**（min 2.010）；同路段 1× 速无干预参照 median 1.993（min 1.823）；**0 次低分告警** ⇒ 无残留偏差 |
| D | **车还没动就先发位姿**（`initial_pose_frame: lidar`，用 `index.txt` 的 `start` 值） | `init with external pose`，score **2.1896**（不是 `init with fp: start`）⇒ 启动前也生效，并证明 `lidar` 语义开关有效 |
| E | 新增脚本 `send_initial_pose.sh`（`--from-tf` 与 `--x --y --yaw` 两种） | 均正确发布并被定位接受（`init with external pose`） |

### 12.7 注意事项与可调参数

1. **点和朝向要尽量准**：这不是全局重定位，而是"以你点的位姿为初值做配准"。
   车在动时请按"你看到车所在的位置"点；1 m 量级的偏差由 12.4 的搜索/精修吸收，再大可能被门槛拒掉
   （日志 `init confidence too low`），**重新点一次更准的即可**，定位不会被带飞。
2. **重复走廊的固有多义性**：本 bag 是长直走廊，若点在完全错误的位置，NDT 有可能匹配到"看起来相似"的
   另一段（实测故意点错后回退功能点 `start`，曾以 1.843 锁到 x≈0 处）。请核对 RViz 中的车位置；
   必要时**再点一次**；希望更严格可调高 `lidar_loc.min_init_confidence`。
3. **可调参数**（`config/default_airy96.yaml`，均为可选键，缺键时用代码默认值 ⇒ 旧 yaml 也能跑新二进制）：

   | 键 | 默认 | 作用 |
   | --- | --- | --- |
   | `common.initial_pose_topic` | `/initialpose` | 话题名；置 `""` 关闭该功能 |
   | `common.initial_pose_frame` | `vehicle` | `vehicle`=车辆 base_link 系（RViz 语义）；`lidar`=`T_map_lidar`（同 `--init_pose`） |
   | `lidar_loc.manual_init_rough_search` | `true` | 手动种子是否启用"粗分辨率 + 小范围 yaw"兜底；若发现误锁定可设 `false` 退回严格模式 |
   | `lidar_loc.manual_init_yaw_search_deg` / `_step` | `30.0` / `7` | yaw 搜索范围(度)与采样数（7 ⇒ 步长 10°） |
   | `lidar_loc.manual_init_refine_iter` | `2` | 接受后再迭代精修次数（消除"附近局部最优"） |

4. **向后兼容**：新增 `YAML_IO::HasValue()`（`yaml_io.h/.cc`）用于探测可选键，避免旧配置因缺键抛 yaml
   异常；普通初始化（功能点 `start` / `recover`）行为与本章改动前完全一致。
5. **离线定位（`run_loc_offline`）不涉及本功能**：它没有 ROS 话题入口；离线换初值请用
   `run_loc_online --init_pose`（第 11 章）或改地图 `index.txt` 的功能点。
6. **验证脚本/日志**：脚本见 `scripts/global_map_fix_check/send_initial_pose.sh`；
   本轮日志留档 `/tmp/manual_init_final_run.log`（用例 C 的关键片段 `/tmp/manual_init_caseC.txt`）。

---

## 13. 栅格地图话题发布（`/localization/map` → 下游规划/控制）

### 13.1 需求

下游（规划/控制）要用**占据栅格地图**（`nav_msgs/msg/OccupancyGrid`）当代价地图/避障输入，
而本工程原先只把栅格地图**写盘**（`data/<地图名>/map.pgm` + `map.yaml`，见 `slam.cc:SaveMap`），
外部进程必须自己解析 PGM+YAML、还得处理行序。需求：

> 把**已保存的栅格地图**按固定间隔**持续**发布成话题 **`/localization/map`**（默认 1 秒一帧）；
> **话题名 / 间隔 / 开关**都能在配置文件中配置。

**发布端放在哪个进程（2026-10-01 修正）**：第一版做在 `run_slam_online`（建图进程），功能可用
（旧数据见 13.5 末尾"方案 A（已废弃）"），但**位置不对**：

| 维度 | 建图进程 `run_slam_online` | 定位进程 `run_loc_online`（现方案） |
| --- | --- | --- |
| 生命周期 | 只在建图期间存在，建完即退出 | 车长期运行期间**一直在跑** |
| 下游何时需要地图 | 建完图之后（车恢复运行） | 与定位同时、且**全程**需要 |
| 数据源 | 正在建的地图 / 已保存地图 | 定位正在使用的那份地图（**同源**） |

⇒ 现方案：**建图端不发布**（照旧只把地图存盘，源码里相关代码已全部撤掉），
**定位端 `run_loc_online` 发布** `<map_path>/map.pgm` + `map.yaml`
（即定位启动时加载、定位匹配所用的那一份），并按**文件 mtime 自动重载**
（重新建图保存后无需重启定位进程）。
原先的 `source` 选项（`saved`/`live`）随建图端一起**取消**：定位端没有"正在建的栅格地图"，
只发已保存的地图文件，语义唯一。

### 13.2 实现（改了什么）

| 文件 | 改动 |
| --- | --- |
| `src/core/system/loc_system.h` | 新增 `LocSystem::MapPublishOptions`（`enable/topic/interval/path/frame_id`）、`nav_msgs/msg/occupancy_grid.hpp` 等头文件、发布者/线程/退出标记、3 个私有方法 |
| `src/core/system/loc_system.cc` | `Init()` 读 `system.publish_map.*`（**全部可选键**，`YAML_IO::HasValue` 探测），创建 `OccupancyGrid` 发布者并启动 `MapPublishLoop()` 线程（在 `loc_->Init()` **之前**启动）；析构时先置退出位并 `join` 再 `loc_->Finish()`；新增 `MapPublishLoop()` / `LoadGridMsg()` / `ResolveMapDir()` |
| `src/core/system/slam.h` / `.cc` | **撤销**：建图端不再有任何发布代码（`Init()` 里只读它自己的 `system.*` 开关；`SaveMap` 只写盘） |
| `src/io/yaml_io.h` / `.cc` | 新增**三层** `HasValue(node_1, node_2, key)`：`system.publish_map.enable` 这类嵌套可选键必须先探测再取值（`GetValue` 缺键会抛 yaml 异常）。另有健壮性修复见 13.6 |
| `config/default_airy96.yaml` | 新增 `system.publish_map:` 段（5 个带注释的可选键；无 `source`） |
| `scripts/global_map_fix_check/gs_map_topic_check.py` | 新增自检脚本（频率/元数据/与 `map.pgm` 逐格比对/轨迹行序判据） |

设计要点：

1. **用独立线程，不用 `rclcpp` timer**：一帧消息要遍历整张栅格表（本平台 2336×512 = **119.6 万格**），
   放在 spin 线程里会挡住 IMU/点云回调（定位掉帧）。线程每 `interval` 秒发一帧，
   睡眠切成 **50 ms** 片，Ctrl-C 后能及时退出并被 `join`（实测日志 `[map pub] 栅格地图发布线程退出`）。
   线程在 `loc_->Init()` **之前**启动 ⇒ 地图还没加载完就已经在发，下游不必等定位收敛。
2. **数据源 = 已保存的栅格地图文件**：读 `<path>/map.pgm` + `map.yaml`
   （`path` 留空时复用定位端的 `system.map_path`，即**下游看到的地图 == 定位用的地图**）；
   按**文件 mtime** 判断更新 ⇒ 重新建图 + 保存地图后**自动换新图，无需重启定位**；
   文件没变时发内存缓存，不重复读盘。
3. **不做发布者降级**：`enable=false` 时既不建发布者也不建线程；地图文件缺失时**只告警一次**、
   不发布、不崩溃（定位照常）。
4. **向后兼容**：整段都是可选键（`YAML_IO::HasValue` 探测，缺键走代码默认值）⇒ 旧 yaml 可直接跑新二进制；
   `interval` 有 **0.05 s 下限保护**（配 0 不会退化成死循环占满 CPU）。
5. **只对在线定位入口生效**：`run_loc_offline` 没有 ROS 节点，不受影响（离线逻辑一行未动）。

### 13.3 坐标系与行序约定（最容易搞错的地方）

消息内容：`info.resolution = g2p5.grid_map_resolution`（本平台 0.05 m）、
`info.origin` = **`data[0]`（栅格左下角）在世界系下的坐标**、`header.frame_id = map`
（可用 `frame_id` 键改）；`data` 用标准语义：**`100` = 占据、`0` = 空闲、`-1` = 未知**。

`LoadGridMsg()` 读图的实现是 `SaveMap` 写图逻辑的**严格逆变换**，这两个约定必须成对记住（改一处要同步改另一处）：

| 环节 | 约定 |
| --- | --- |
| `SaveMap` 写 `map.pgm`（`slam.cc`） | `nav_image(height-1-y, x) = (data==0 ? 255 : data==100 ? 0 : 128)` |
| `LoadGridMsg` 读回（`loc_system.cc`） | `data[y][x] ← pgm(height-1-y, x)`；`occ = (255-v)/255`（`negate=0`），`occ > occupied_thresh ⇒ 100`、`occ < free_thresh ⇒ 0`、其余 `-1` |

即：**图像第 0 行 = 地图顶部（y 最大）**，而 `OccupancyGrid.data` 从**左下角（origin）**起按行排。
这与 `scripts/global_map_fix_check/gs_g2p5_check.py::occupied_world_xy()` 里的既有写法
（`y = origin[1] + ((h-1-rows)+0.5)*RES`，早前已用 `global.pcd` 校验过）**完全一致**。

### 13.4 配置与用法

`config/default_airy96.yaml` → `system.publish_map`（全部为可选键，缺键走代码默认值）：

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `enable` | `true` | 开关（`false` ⇒ 不建发布者也不建线程） |
| `topic` | `/localization/map` | 话题名 |
| `interval` | `1.0` | 发布间隔(秒)；下限保护 0.05 s |
| `path` | `""` | 栅格地图目录（空 ⇒ `system.map_path` ⇒ `./data/<地图名>/`） |
| `frame_id` | `map` | 消息 `header.frame_id` |

```bash
# 起在线定位（默认即开启发布）
ros2 run lightning run_loc_online --config src/lightning-slam/config/default_airy96.yaml

# 下游侧
ros2 topic hz   /localization/map                       # ~1 Hz
ros2 topic echo /localization/map --once --field info    # 只看元数据（整条 echo 太大）
# RViz2：Fixed Frame = map，Add → Map → Topic = /localization/map

# 自检脚本（频率 + 元数据 + 与 map.pgm 逐格比对 + 轨迹行序判据）
python3 scripts/global_map_fix_check/gs_map_topic_check.py --dur 6 \
        --ref data/new_map --ref-log /tmp/run_slam_online.INFO
```

### 13.5 实测验证（2026-10-01 23:29~23:42 手工逐项；23:52~23:56 用一键脚本复跑，结果一致）

> 环境同第 11/12 章：工作目录 `/home/h/ddh/code/ws/lightning_ws`，地图 `./data/new_map`
> （2336×512 @0.05 m，origin(-9.85,-17.85)；`0.pcd`/`1.pcd`/`global.pcd`），
> bag `follow_car/20260914/decompressed/rosbag2_2026_09_15-10_22_04`。
> 被测进程：**`run_loc_online`**（默认 config 即开启发布）；检查脚本
> `scripts/global_map_fix_check/gs_map_topic_check.py`（订阅采样 + 与 `map.pgm` 逐格比对 + 轨迹判据）。
>
> **以下 L1~L8 已固化成一键复现脚本**
> `bash scripts/global_map_fix_check/run_map_topic_checks.sh`
> （自动生成变体 yaml（`/tmp/map_topic_checks/`）与 L5 的合成地图（`/tmp/map_topic_alt/`），
> 全程约 3~4 分钟；本轮实测输出 `/tmp/mt_run2.log`，逐项日志 `*.chk.log`）。
> 脚本退出前后均 `kill_all`，结尾打印 `全部残留进程(应 0)` 与 `L1_L8_DONE`；
> 其中 L2/L6/L8 的检查脚本**退出码 1 属预期**（"0 帧"正是要验证的结果）。

**L1) 基本可用性（默认 config；不喂传感器数据也照发）**

| 检查项 | 结果 |
| --- | --- |
| 启动日志 | `栅格地图发布已开启: topic = /localization/map, interval = 1 s, frame_id = map, map_dir = ./data/new_map`；`[map pub] 已加载栅格地图 ./data/new_map/map.pgm (2336 x 512 @ 0.05 m, origin -9.85, -17.85)` |
| 话题 | `ros2 topic list` 有 `/localization/map`；`ros2 topic info` = `nav_msgs/msg/OccupancyGrid`，Publisher count 1 |
| 频率 | 6.0 s 收到 **6 帧** ⇒ **0.998~0.999 Hz**（帧间隔 中位 0.999~1.003 s）；首帧延迟 ≈0.7 s（订阅 → 第一帧，首次要读盘 119.6 万格） |
| 元数据 | `2336×512 @ 0.050 m, origin=(-9.85,-17.85), frame_id="map"` —— 与 `map.yaml` **逐项一致** |
| 内容 | 与 `map.pgm` **全图逐格比对（1 196 032 格）**：**同格一致率 = 1.0000**、**占据↔空闲互翻 = 0 格**；占据/空闲/未知 = 10860 / 260058 / 925114（与参考完全相同） |
| 行序/朝向（独立判据） | 用建图日志的 **2376 个车辆轨迹点**：正确行序下 **99.5%** 落在"空闲"格（100% 落在地图内）；**故意上下翻转后 0.0%**（全落"未知"）⇒ 行序正确 |
| 退出 | SIGINT 后打印 `[map pub] 栅格地图发布线程退出`，无残留进程 |

**L2) 开关（`enable: false`）**：日志 `栅格地图发布已关闭 (system.publish_map.enable = false)`；
`ros2 topic list` 里**没有** map 相关话题（也不建发布者/线程）；订阅 5 s **0 帧**。
> 查 `topic list` 前先 `ros2 daemon stop`：守护进程里可能还缓存着上一轮定位端注册过的话题名，
> 不清缓存会看到"假阳性"。

**L3) 兼容旧 yaml（整段删掉 `publish_map`）**：正常启动（**无** `YAML::InvalidNode` 崩溃），
按**默认值**发布（`/localization/map`、1 s、`map`）：**0.999~1.002 Hz**、一致率 **1.0000 / 互翻 0 格**
⇒ 5 个键缺省全可用（`YAML_IO::HasValue` 安全下钻，见 13.6）。

**L4) 话题名/间隔/frame_id 可配**：`topic=/map_custom`、`interval=0.2`、`frame_id=custom_map` ⇒
启动日志同步显示三项（`topic = /map_custom, interval = 0.2 s, frame_id = custom_map`）；
实测 **≈5 Hz**（两次：4.982 / 5.169 Hz，帧间隔 中位 0.200 s）、末帧 `frame_id="custom_map"`、
一致率 1.0000 / 互翻 0 格。同时旧话题 `/localization/map` 不再出现（整轮只发 `topic` 指定的名字）。

**L5) 地图目录可覆盖（`path`）**：把 `path` 指到 `/tmp/map_topic_alt`（合成一张 10×10 @0.1 m、
origin(-5,-5)、占据 16 格 / 空闲 84 格的 map_server 地图，P5 二进制 —— 与 `SaveMap` 写出的格式一致）⇒
启动日志 `map_dir = /tmp/map_topic_alt`、
`已加载栅格地图 /tmp/map_topic_alt/map.pgm (10 x 10 @ 0.1 m, origin -5, -5)`，末帧 `10x10 @ 0.100 m`、
与那张合成图**一致率 1.0000 / 互翻 0 格**（100 格全覆盖）；**同时**定位仍按 `system.map_path` 加载
`data/new_map`（日志 `tiled_map.cc:170] loaded chunks: 2`，而合成图目录里根本没有 `*.pcd`）
⇒ 两个键**各管各的**：`map_path` 决定"拿哪张图定位"，`publish_map.path` 决定"发哪张图"。

**L6) 地图文件缺失**：`path=/tmp/map_topic_missing`（该目录故意不存在）⇒ **只告警一次**
`[map pub] 未找到已保存的栅格地图 /tmp/map_topic_missing/map.pgm（检查 system.map_path / system.publish_map.path 是否指向建图保存的目录）`，
订阅 5 s **0 帧**；进程存活、定位节点照常创建（`online loc node has been created`）⇒ 不崩、不退出、
不影响定位。

**L7) 与定位并行跑真实数据链路**：`run_loc_online` + 同一 bag **0.5 倍速播 40 s** ⇒
定位侧正常：`start loc without external init pose, use map functional point 'start' instead` →
`fitness_score is: 2.18774, global_pose is: 0.00398934 0.000882326 -4.77317e-06` → `init with fp: start`，
期间定位分值帧均值 **2.162 / 2.143**（两次独立运行，帧数 12 / 23；与第 11/12 章同量级，无退化）；
**同时**话题照发：6 s 收到 **6 帧 = 0.996~0.999 Hz**（帧间隔 中位 1.002 s）、全图一致率 1.0000 / 互翻 0 格
⇒ 发布线程与定位主循环解耦，**不抢帧、不掉频**（地图文件没变时不重复读盘）。播完退出无残留进程。

**L8) 回归：建图端确实不再发布**：`run_slam_online`（`with_ui=false`, `--config /tmp/map_topic_checks/slam.yaml`）
起来后（日志 `online slam node has been created`）：发布相关日志行数 **0**；
清掉 ROS 守护进程缓存后 `ros2 topic info /localization/map` = **`Unknown topic`**、
`ros2 topic list` 里命中数 **0**、`gs_map_topic_check.py` 订阅 5 s **0 帧**
⇒ 建图端没有任何残留的发布者/线程。

**方案 A（已废弃，留档）**：第一版做在 `run_slam_online` 上（可选 `saved`/`live` 两种源）。
当时实测：`saved` 源同样 **0.996~0.998 Hz**、逐格一致率 **1.0000 / 互翻 0 格**、回环一致；
`live` 源 **0.998 Hz**，栅格随关键帧正常扩张（1120×224 → 1296×304），与"落盘快照"的往返一致率
0.9934~0.9768（差异随时差增长，属正常）。按 13.1 的结论，`run_slam_online` 里的发布代码已**整体撤掉**；
其中"读 `map.pgm` + `map.yaml` 并逐格转 OccupancyGrid"的加载器原样搬到定位端
（`LocSystem::LoadGridMsg()`，含 `negate/thresh` 处理与行序翻转），因此上面的 `saved` 结论对现方案
依然成立 —— 并且已在定位端用 L1/L3/L4/L5/L7 **重新验证**（数值完全一致）。

### 13.6 顺带修掉的缺陷：`YAML_IO::HasValue()` 在"缺整段配置"时会抛异常（影响面超出本功能）

**症状**（本功能的"旧 yaml 兼容"用例暴露的）：把 `system.publish_map` **整段**删掉后启动即崩：

```
terminate called after throwing an instance of 'YAML::InvalidNode'
  what():  invalid node; first invalid key: "publish_map"
```

**根因**（用最小程序 `/tmp/ymltest.cc` 实测 yaml-cpp）：

| 表达式 | 行为 |
| --- | --- |
| `validMap["missing_key"]` | 返回**无效节点**（不抛） |
| `invalidNode.IsDefined()` / `.IsNull()` | **安全**，返回 false |
| `invalidNode.IsMap()` / `.as<T>()` | **抛 `YAML::InvalidNode`**（消息里带"first invalid key"） |

原实现是 `const YAML::Node n2 = n1[node_2]; if (!n2.IsMap()) return false;` ——
键缺失时 `n2` 是无效节点，紧接着的 `IsMap()` 就抛异常 ⇒ 进程崩溃。

**修复**：`yaml_io.cc` 新增 `GetChildNode()`，按 **`IsDefined/IsNull` → `IsMap` → `operator[]`** 的顺序安全下钻，
两个 `HasValue`（两层 / 三层）都改为调它（`source_`/`topic` 之类的 `GetValue` 仍在 `HasValue` 为真之后才调用）。

**影响面（重点）**：本功能新增的是**三层**键（`system.publish_map.*`），"整段缺失"会直接崩（13.5-L3 实测到）；
**两层**版 `HasValue` 也有同一模式的隐患：当**父段整个缺失**时 `if (!sub.IsMap())` 同样会对无效节点抛异常
（例如 `common` / `system` 段不存在、或层级不是 map）。本次两个版本一并改为安全写法，
回归用例：`run_loc_online` 用**删掉** `common.initial_pose_topic` / `initial_pose_frame` 的 yaml 启动，
不再抛异常、功能自动关闭（日志 `manual init pose topic: /initialpose` 即走默认值）；
三层版用"整段删掉 `publish_map`"验证（13.5-L3）。**建议其它新增可选键一律用 `HasValue()` 探测，
不要直接用 `yaml[...][...].as<T>()`。**

### 13.7 注意点

1. **发布的是"定位正在用的那张图"**：`publish_map.path` 留空 ⇒ 取 `system.map_path`（默认
   `./data/<地图名>/`），与定位加载的 `map.pgm`/`*.pcd` 同一份 ⇒ 下游看到的地图不会和定位"错版"。
   要发别的图（例如给另一片区域）才需要显式写 `path`。
2. **带宽**：本平台整张图 119.6 万格 ⇒ 单帧约 **1.2 MB**，1 Hz ≈ **1.2 MB/s**；地图更大或频率更高时
   请评估网络/下游反序列化开销（把 `interval` 调大即可）。
3. **QoS**：发布端 `rclcpp::QoS(1)`（reliable / volatile / depth 1），下游用默认 QoS 即可收到；
   如需"晚加入也立刻拿到最后一帧"，可自行改 `transient_local`（本功能未开，避免与下游既有配置冲突）。
4. **重载条件**：只看 `map.pgm` 的 **mtime**；建图保存时是 `remove_all` + 重建目录 ⇒ mtime 必变，
   所以"重新建图 → 保存地图"后定位端会自动换新图（日志会再打一行 `已加载栅格地图 ...`）。
   地图目录填相对路径时依赖**进程启动目录**（建议按第 4 章说明改绝对路径）。
5. **`run_slam_online` / `run_slam_offline` 都不发布**：栅格话题只由 `run_loc_online` 提供；
   建图端仍然只负责把地图写盘（`data/<地图名>/map.pgm`）。离线定位入口没有 ROS 节点，同样不涉及。
6. **发布线程与主流程解耦**：线程只读（读盘文件 / 用 mtime 缓存），不碰定位内部状态；
   即使一个订阅者都没有，也只是空发（无副作用）；线程在 `loc_->Init()` **之前**启动 ⇒ 地图还没加载完
   就已经能发，下游启动顺序不受约束。
---

## 14. TF 树：定位改发 `map→odom`，与车上已有 TF 树合并（2026-10-01 新增）

### 14.1 问题：`map→base_link` 与车上 TF 冲突

车（本平台实测，数据取自 rosbag 的 `/tf` 与 `/tf_static`）本来就有一套 TF 树：

```
odom --(动态, /tf)--> base_footprint --(静态, /tf_static)--> base_link --(静态)--> laser_points_front/rear、uwb_*
```

而 `run_loc_online` 原来把定位结果**直接发成 `map→base_link`**
（`localization_result.cc:ToGeoMsg()`：`header.frame_id = "map"`、`child_frame_id = "base_link"`），
于是 `base_link` 同时有**两个父节点**（`base_footprint` 与 `map`）⇒ **TF 树冲突**：

- RViz / Nav2 报 multiple parents（或只认其中一条），map 与 odom 两棵树在画面上互相打架；
- 下游按 `map→base_link` 与 `odom→base_link` 分别查询，会得到互相不一致的结果（差一个 `T_map_odom`）。

### 14.2 正确的分工与数学

定位只负责"世界系那一段"，即补 **`map→odom`** 这一条边：

```
T_map_odom = T_map_base · (T_odom_base)⁻¹
```

- `T_map_base`：定位结果（`ToGeoMsg()`，已是**车辆 base_link 系**，见第 11 章的外参换算）；
- `T_odom_base`：**车上已有**的 `odom→base_link`，用 tf2 查询
  （`odom→base_footprint→base_link` 这类中间帧由 tf2 自动串联）；
- 发出去的 `map→odom` 就是**里程计系的漂移修正量**（车静止时缓慢变化；
  本平台实测初始约 `t = (0.529, -1.165, -0.083) m` —— odom 原点与 map 原点的差）。

最终 TF 树完整且无冲突：

```
map → odom → base_footprint → base_link → laser_points_front/rear、uwb_*
```

下游以 `map` 为 fixed frame 查 `base_link`，得到的**仍然是定位位姿**（与旧行为数值等价），
只是不再多出一个父节点 —— 这正是 RViz/Nav2 期望的接法。

### 14.3 实现（改了什么）

| 位置 | 改动 |
| --- | --- |
| `loc_system.h` | 新增 `TfOptions`（`enable / mode / map_frame_id / odom_frame_id / base_frame_id / lookup_timeout`）、`tf_buffer_`、`tf_listener_`、`PublishTf()`、`StartTfListener()`，并把原来那句"直接 `sendTransform`"的回调换成 `PublishTf()` |
| `loc_system.cc:Init()` | 解析 `system.tf.*`（**全部是可选键**，用 `YAML_IO::HasValue()` 探测）；`enable` 时建 broadcaster，`mode=map_odom` 时再建 tf2 监听 |
| `loc_system.cc:PublishTf()` | 定位结果回调：先查 `odom→base_link`，合成 `map→odom` 后发布；"直接发 `map→base_link`"的旧路径只在 `mode=map_base` 时保留 |
| `loc_system.cc:StartTfListener()` | `tf2_ros::Buffer`（缓存 10 s，传 `node_` 以支持带超时查询与 `view_frames` 调试服务）+ `tf2_ros::TransformListener` |
| `loc_system.cc:~LocSystem()` | 在 `loc_->Finish()`（定位线程已停、回调不会再进来）**之后**才拆 TF，避免回调访问已析构对象 |
| `config/default_airy96.yaml` | 新增带注释的 `system.tf` 段（整段删掉也能跑，走代码默认值） |

关键实现细节：

1. **查询失败时不退回 `map→base_link`**：宁可本帧不发 `map→odom`（并打 5 s 节流的 WARNING），
   也不制造双父节点。旧行为只在显式配置 `system.tf.mode: "map_base"` 时才启用。
2. **时标**：优先按**定位结果的时间戳**查（tf2 会插值，精度最高）；查不到（刚启动、
   车上 TF 时标滞后等）退化为"取最新可用"；仍失败则跳过本帧。
3. **不阻塞定位输出**：`lookup_timeout` 默认 0（只查缓存、不等待）。原因是 TF 回调运行在
   **定位工作线程**（`AsyncMessageProcess`）里，等待会拖慢定位；车上 TF 连续发布，
   稳态下按时间戳插值即可命中。确有需要时可设 `0.02~0.05`。
4. **监听自带线程**：`TransformListener` 默认 `spin_thread=true` —— 它把 `/tf`、`/tf_static`
   放进一个**独立 callback group + 独立线程**里 spin（`automatically_add_to_executor_with_node=false`），
   因此**不会**与本工程主节点那句单线程 `spin(node_)` 打架，也不需要额外 spin。

### 14.4 配置与用法

`config/default_airy96.yaml` 的 `system` 段新增（**全部可选，整段删掉也能跑**）：

```yaml
  tf:
    enable: true                  # 总开关；false => 不发 TF、也不建 tf2 监听
    mode: "map_odom"              # map_odom(默认) = 发 map→odom（需要车上已有 odom→base_link）
                                  # map_base        = 老行为，直接发 map→base_link（会有双父节点）
    map_frame_id: "map"           # 地图帧名（发布出去的父帧）。正常情况下定位结果自带 frame_id="map"，
                                  #   以结果里的为准；本项只在结果帧名为空时兜底
    odom_frame_id: "odom"         # 里程计帧名（必须与车上 TF 一致；发布出去的子帧）
    base_frame_id: "base_link"    # 车体帧名（查 odom→base_link 的目标帧；child_frame_id 为空时兜底）
    lookup_timeout: 0.0           # 查 odom→base_link 的最长等待(秒)；默认 0 = 只查缓存不等待
```

```bash
# 起在线定位（默认即发 map→odom）
cd /home/h/ddh/code/ws/lightning_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lightning run_loc_online --config src/lightning-slam/config/default_airy96.yaml

# 自检（父帧唯一性 + map→odom 合成自检 + 整棵树连通）
python3 src/lightning-slam/scripts/global_map_fix_check/gs_tf_tree_check.py --dur 8
#   → 期望：base_link 只有一个父帧 base_footprint、odom <- map、
#           map→base_link = (map→odom)·(odom→base_link)（同一时刻查询，最大差 < 1e-6）

# 目视：RViz2 Fixed Frame 填 map，按 TF 显示；或
ros2 run tf2_tools view_frames          # 生成 frames.pdf（定位端还顺带提供了 tf_frames 服务）
ros2 run tf2_ros tf2_echo map base_link # 与 map→odom + odom→base_link 应一致
```

### 14.5 实测验证（2026-10-01，用户自己的 bag，全程约 35 s 短测）

数据：bag `/home/h/ddh/bag/930pm/bag/jieya/rosbag2_2026_10_01-00_49_15`
（27.3 s，含 `/tf` 1367 条、`/tf_static` 8 条、`/lidar/front/imu`、`/lidar/front/points_raw`），
地图 `lightning_ws/data/new_map`，默认 config，脚本 `scripts/global_map_fix_check/gs_tf_tree_check.py`。

| 阶段 | 做法 | 结果 |
| --- | --- | --- |
| 1（默认 `map_odom`） | bag + `run_loc_online`（默认 config）→ 采样 8 s | 日志 `tf 监听已建立（订阅 /tf、/tf_static）…`、`tf 发布已开启: map→odom …`、`[tf] 已开始发布 map→odom … t = (0.529, -1.165, -0.083)`；父帧一览 **`odom <- map`、`base_footprint <- odom`、`base_link <- base_footprint`、`laser_points_*/uwb_* <- base_link`**；A) `base_link` 父帧 = `['base_footprint']` **唯一** ✓；B) 同一时刻 1790780164.318 查询，`map→base_link = (map→odom)·(odom→base_link)` 最大差 **3.33e-16** ✓；C) `map→base_link` 解析成功 `t = (-0.049, -0.047, 0.047) m, yaw = 6.61°` ✓ → **通过**（退出码 0）。改完最后一行代码后**复跑一次**（`/tmp/tf_smoke4.log`）：2008 条 TF、合成最大差 8.05e-16、`t = (-0.108, -0.054, 0.079) m, yaw = 6.37°`，结论一致（位姿数值随播包时刻略有差异属正常） |
| 2（反证：判据有区分度） | 额外用 `ros2 topic pub -r 10 /tf` 发一条 `map→base_link` | 检查脚本报 `✗ base_link 的父帧里出现了 map ⇒ TF 树冲突`、`✗ 查不到 map→odom` → **未通过**（退出码 1），说明判据能抓住旧行为 |
| 3（兼容回归 `map_base`） | 把 `system.tf.mode` 改成 `map_base` 再跑一次 13 s + 采样 6 s | 日志 `tf 发布已开启: map→base_link（system.tf.mode = map_base，老行为…）`、**没有** `tf 监听已建立`；父帧 `base_link <- base_footprint, map`（= 老行为双父节点）；`--mode map_base` 下检查脚本通过（退出码 0） |

其它观察：

- 定位功能未受影响：日志 `fitness_score is: 2.55898` → `init with fp: start`，与第 11 章的量级一致；
  整个短测期间定位正常出结果（`[tf] 已开始发布 map→odom` 即由定位结果驱动）。
- bag 播完后（车上 `/tf` 停止）日志出现节流告警
  `W1002 00:32:31 [tf] 查不到车上已有的 odom→base_link 变换，本帧不发布 map→odom …`
  —— 这正是 14.3-1 的设计行为（不退回 `map→base_link`）。
- 三阶段结束均 `残留进程(应 0): 0`。

### 14.6 注意点

1. **车上必须有 `odom→base_link`**（本平台是 `odom→base_footprint`(动态) + `base_footprint→base_link`(静态)，
   tf2 会自动串联）。若你的车没有这条链，日志会周期性出现"查不到 odom→base_link"的 WARNING，
   此时要么补齐里程计 TF，要么显式把 `system.tf.mode` 设为 `map_base`（回到旧行为，接受双父节点）。
2. **帧名要与车上一致**：`odom_frame_id` 默认 `odom`；若车上是 `odom_combined` 之类，改这一项即可
   （`map_frame_id` / `base_frame_id` 同理；发布出去的父帧号会跟着变）。
3. **不要同时跑两个定位进程**：两个进程都会发 `map→odom`，会互相覆盖（TF 只认一条边）。
   切换/重启定位时请先确认上一个进程已退出。
4. **与 RViz "2D Pose Estimate" 配合**：手动初始位姿仍在 `map` 系下（语义 = 车辆 base_link 在 map 下的位姿），
   与新的 `map→odom` 发布方式无关，第 12 章的用法不变。
5. **时标差**：`map→odom` 用的是定位结果的时间戳（= 雷达帧时刻），而 `odom→base_link` 来自底盘/里程计；
   两者差几十毫秒是正常的（下游按 `map→base_link` 查询时 tf2 会各自插值），
   若发现 RViz 里地图相对车有轻微抖动，可先把 `lookup_timeout` 设为 `0.02` 观察（默认 0 不等待）。

---

*报告生成环境：`/home/h/ddh/code/ws/lightning_ws/src/lightning-slam`，
数据 bag `/home/h/ddh/bag/1/rosbag2_2026_09_30-20_38_07`（308 帧）。
第 1~9 章：`GetGlobalMap` 补丁（运行目录 `/tmp/gs_fix_run`，2026-10-01 18:36 构建）；
第 10 章：定位端 + g2p5 补丁（运行目录 `/tmp/g2p5fix_run`、`/tmp/loc_fixrun`，2026-10-01 19:20 构建）；
第 11 章：在线定位入口补丁（`run_loc_online` 初值坐标系 + 初始化门槛，2026-10-01 20:40 构建；
验证链路 = 用户原始命令 + `follow_car/20260914/decompressed/rosbag2_2026_09_15-10_22_04` 全程 238 s
+ 用户地图 `lightning_ws/data/new_map`，日志 `/tmp/loc_bagB_fixed.log`、`/tmp/loc_bagB_full.log`）。
第 12 章：手动初始位姿新功能（`/initialpose` ← RViz 2D Pose Estimate，2026-10-01 21:52 构建；
验证链路 = 同一 bag 以 0.5 倍速播放（数据连续、不暂停）+ 用户地图，
用例 A/B/C/D/E 日志 `/tmp/mi_run.log`（留档 `/tmp/manual_init_final_run.log`））。
第 13 章：栅格地图话题发布新功能（**定位端** `/localization/map`，2026-10-01 23:28 构建；另有 22:58 的
`YAML_IO::HasValue()` 健壮性修复重建。第一版曾做在 `run_slam_online`，同日 23:2x 按 13.1 的结论
**整体撤销并搬到 `run_loc_online`**）。验证链路 = 默认 config + 用户地图 `data/new_map`、
bag `follow_car/20260914/decompressed/rosbag2_2026_09_15-10_22_04` 0.5 倍速（L7 并行用例），
脚本 `scripts/global_map_fix_check/gs_map_topic_check.py`，
日志 `/tmp/lo3_out.txt`（L5~L8）、`/tmp/lo4_out.txt`（L8 复核）、`/tmp/lo_l{1..8}.log`；
L1~L8 已固化为一键复现脚本 `scripts/global_map_fix_check/run_map_topic_checks.sh`
（2026-10-01 23:52~23:56 复跑一次，输出 `/tmp/mt_run2.log`，与手工逐项结果一致，
结尾 `残留进程(应 0): 0` + `L1_L8_DONE`）。
第 14 章：TF 树修复（定位改发 `map→odom`，2026-10-02 00:32 构建；另有最后一行代码微调后的 00:4x 重建）。
验证链路 = 用户 bag `930pm/bag/jieya/rosbag2_2026_10_01-00_49_15`（27.3 s，1 倍速播放）+ 用户地图
`lightning_ws/data/new_map`，脚本 `scripts/global_map_fix_check/gs_tf_tree_check.py`，
日志 `/tmp/tf_smoke.log`、`/tmp/tf_smoke2.log`、`/tmp/tf_smoke4.log`（阶段1 通过 + 阶段2 反证失败）、
`/tmp/tf_smoke3.log`（阶段3 `map_base` 兼容回归），定位日志 `/tmp/tf1_loc.log`、`/tmp/tf3_loc.log`，
三阶段结束均 `残留进程(应 0): 0`。



