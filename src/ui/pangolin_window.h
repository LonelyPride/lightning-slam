#pragma once

#include <map>
#include <memory>
#include <queue>

#include "common/eigen_types.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "common/nav_state.h"
#include "common/point_def.h"

// -----------------------------------------------------------------------------
// 个人修改（2026-10-09，aarch64 交叉编译支持）：UI 有"真 Pangolin"和"空实现"两套，
// 由 CMake 统一定义的 LIGHTNING_WITH_UI 选择（见 cmake/packages.cmake）：
//   LIGHTNING_WITH_UI=1 → 下面的真 PangolinWindow（依赖 Pangolin；本机构建默认）
//   LIGHTNING_WITH_UI=0 → 文件末尾的空实现（交叉环境 sysroot 里没有 Pangolin）
//
// 两套的【公开接口完全一致】，因此 src/ 里所有调用点都不用改：
//   - 调用点本来就写成 `if (ui_) { ui_->Xxx(); }`（见 localization.cpp / slam.cc /
//     laser_mapping.cc / lidar_loc.cc）；
//   - LIGHTNING_WITH_UI=0 时这些调用打到"什么都不做"的空实现上。
// 切到空实现后，`system.with_ui: true` 会失效（Init() 里打一条 WARNING 提示），
// 上板定位（viz:=false）不受影响。
//
// 注意：宏在 CMake 里一定有值；这里给个默认 1 只是让不走 CMake 的编辑器/索引
// 也能解析本文件（保持与本机构建一致）。
// -----------------------------------------------------------------------------
#ifndef LIGHTNING_WITH_UI
#define LIGHTNING_WITH_UI 1
#endif

#if LIGHTNING_WITH_UI

namespace lightning::ui {

class PangolinWindowImpl;

/**
 * @note 此类本身不直接涉及任何opengl和pangolin操作，应当放到PangolinWindowImpl中
 */
class PangolinWindow {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PangolinWindow();
    ~PangolinWindow();

    // @brief 初始化窗口，后台启动render线程。
    // @note 与opengl/pangolin无关的初始化，尽量放到此函数体中;
    //       opengl/pangolin相关的内容，尽量放到PangolinWindowImpl::Init中。
    bool Init();

    void Reset(const std::vector<Keyframe::Ptr>& keyframes);

    // 更新激光地图点云，在激光定位中的地图发生改变时，由fusion调用
    void UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud);

    // 更新动态图层点云
    void UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud);

    // 更新kalman滤波器状态
    // 个人修改：state 中的位姿必须是【车辆(base_link)系】姿态，即由调用方先做
    //   state.SetPose(vehicle_frame::ToVehiclePose(T_w_body))，否则 IMU 竖装会让车"绕 y 轴立起来 90°"
    void UpdateNavState(const NavState& state);

    // 更新最新的pose
    void UpdateRecentPose(const SE3& pose);

    void UpdatePredictPose(const SE3& pose);

    // 更新一次scan和它对应的Pose
    // 个人修改：pose 必须是"能把 cloud 直接变换到世界系"的位姿（即 T_w_<cloud所在坐标系>）。
    //   例如雷达系点云应传 T_w_lidar = T_w_body * T_imu_lidar，而不是 IMU 姿态；
    //   画车所需的车辆系姿态请用 UpdateBodyPose() 单独给出。
    void UpdateScan(CloudPtr cloud, const SE3& pose);

    // 个人修改：更新"车"的位姿（画车/三轴箭头用），要求传入【车辆(base_link)系】姿态
    //   T_w_vehicle = T_w_body * Mᵀ（vehicle_frame::ToVehiclePose）。
    //   与 UpdateScan() 分开：UpdateScan() 的 pose 是点云所在坐标系（雷达系）的位姿，
    //   用它画车会让车跟着雷达外参一起歪（Airy 竖装雷达约 90°）。
    void UpdateBodyPose(const SE3& pose);

    void UpdateKF(std::shared_ptr<Keyframe> kf);

    // 等待显示线程结束，并释放资源
    void Quit();

    // 用户是否已经退出UI
    bool ShouldQuit();

    // 设置IMU到雷达的外参
    void SetTImuLidar(const SE3& T_imu_lidar);

    // 设置需要保留多少个扫描数据
    void SetCurrentScanSize(int current_scan_size);

   private:
    std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;
};
}  // namespace lightning::ui

#else  // LIGHTNING_WITH_UI

#include <vector>

#include <glog/logging.h>

namespace lightning::ui {

/**
 * @brief 没有 Pangolin 时用的"空 UI"（LIGHTNING_WITH_UI=0，交叉编译默认）
 *
 * 公开接口与上面真正的 PangolinWindow 完全一致，但每个函数都是空操作：
 * 不创建窗口、不启动渲染线程、不持有任何点云。这样 src/ 里所有 `ui_->Xxx()`
 * 调用点（都写在 `if (ui_)` 里，见 localization.cpp / slam.cc / laser_mapping.cc /
 * lidar_loc.cc）一行都不用改，交叉编译（Jetson 等 aarch64 板）也无需 Pangolin。
 */
class PangolinWindow {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PangolinWindow() = default;
    ~PangolinWindow() = default;

    /// 真 UI 在此创建窗口并启动渲染线程；空 UI 只提示一次并返回 false（无窗口可用）
    bool Init() {
        static bool warned = false;
        if (!warned) {
            warned = true;
            LOG(WARNING) << "this build was compiled without the Pangolin 3D UI (LIGHTNING_WITH_UI=OFF); "
                            "system.with_ui is ignored";
        }
        return false;
    }

    void Reset(const std::vector<Keyframe::Ptr>& /*keyframes*/) {}

    void UpdatePointCloudGlobal(const std::map<int, CloudPtr>& /*cloud*/) {}

    void UpdatePointCloudDynamic(const std::map<int, CloudPtr>& /*cloud*/) {}

    void UpdateNavState(const NavState& /*state*/) {}

    void UpdateRecentPose(const SE3& /*pose*/) {}

    void UpdatePredictPose(const SE3& /*pose*/) {}

    void UpdateScan(CloudPtr /*cloud*/, const SE3& /*pose*/) {}

    void UpdateBodyPose(const SE3& /*pose*/) {}

    void UpdateKF(std::shared_ptr<Keyframe> /*kf*/) {}

    void Quit() {}

    /// 空 UI 永远"已退出"（安全侧：调用方若用它当循环条件，不会空转）
    bool ShouldQuit() { return true; }

    void SetTImuLidar(const SE3& /*T_imu_lidar*/) {}

    void SetCurrentScanSize(int /*current_scan_size*/) {}
};
}  // namespace lightning::ui

#endif  // LIGHTNING_WITH_UI
