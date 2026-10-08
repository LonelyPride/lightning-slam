#pragma once

#include <map>
#include <memory>
#include <queue>

#include "common/eigen_types.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "common/nav_state.h"
#include "common/point_def.h"

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

    /// @brief 初始化窗口，后台启动render线程。
    /// @note 与opengl/pangolin无关的初始化，尽量放到此函数体中;
    ///       opengl/pangolin相关的内容，尽量放到PangolinWindowImpl::Init中。
    bool Init();

    void Reset(const std::vector<Keyframe::Ptr>& keyframes);

    /// 更新激光地图点云，在激光定位中的地图发生改变时，由fusion调用
    void UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud);

    /// 更新动态图层点云
    void UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud);

    /// 更新kalman滤波器状态
    /// 个人修改：state 中的位姿必须是【车辆(base_link)系】姿态，即由调用方先做
    ///   state.SetPose(vehicle_frame::ToVehiclePose(T_w_body))，否则 IMU 竖装会让车"绕 y 轴立起来 90°"
    void UpdateNavState(const NavState& state);

    /// 更新最新的pose
    void UpdateRecentPose(const SE3& pose);

    void UpdatePredictPose(const SE3& pose);

    /// 更新一次scan和它对应的Pose
    /// 个人修改：pose 必须是"能把 cloud 直接变换到世界系"的位姿（即 T_w_<cloud所在坐标系>）。
    ///   例如雷达系点云应传 T_w_lidar = T_w_body * T_imu_lidar，而不是 IMU 姿态；
    ///   画车所需的车辆系姿态请用 UpdateBodyPose() 单独给出。
    void UpdateScan(CloudPtr cloud, const SE3& pose);

    /// 个人修改：更新"车"的位姿（画车/三轴箭头用），要求传入【车辆(base_link)系】姿态
    ///   T_w_vehicle = T_w_body * Mᵀ（vehicle_frame::ToVehiclePose）。
    ///   与 UpdateScan() 分开：UpdateScan() 的 pose 是点云所在坐标系（雷达系）的位姿，
    ///   用它画车会让车跟着雷达外参一起歪（Airy 竖装雷达约 90°）。
    void UpdateBodyPose(const SE3& pose);

    void UpdateKF(std::shared_ptr<Keyframe> kf);

    /// 等待显示线程结束，并释放资源
    void Quit();

    /// 用户是否已经退出UI
    bool ShouldQuit();

    /// 设置IMU到雷达的外参
    void SetTImuLidar(const SE3& T_imu_lidar);

    /// 设置需要保留多少个扫描数据
    void SetCurrentScanSize(int current_scan_size);

   private:
    std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;
};
}  // namespace lightning::ui
