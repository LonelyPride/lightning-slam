#pragma once

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

#include "common/imu.h"
#include "core/lio/laser_mapping.h"
#include "core/localization/localization_result.h"
#include "core/system/async_message_process.h"

// 预声明
namespace lightning {
namespace ui {
class PangolinWindow;
}

namespace loc {

class LidarLoc;
class PGO;

/**
 * 实时定位接口实现
 */
class Localization {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = false;  // 在线模式还是离线模式
        bool with_ui_ = false;      // 是否带ui

        // 参数
        SE3 T_body_lidar_;

        bool enable_lidar_odom_skip_ = false;  // 是否允许激光里程计跳帧
        int lidar_odom_skip_num_ = 1;          // 如果允许跳帧，跳多少帧
        bool enable_lidar_loc_skip_ = true;    // 是否允许激光定位跳帧
        bool enable_lidar_loc_rviz_ = false;   // 是否允许调试用rviz
        int lidar_loc_skip_num_ = 4;           // 如果允许跳帧，跳多少帧
        bool loc_on_kf_ = false;
    };

    Localization(Options options = Options());
    ~Localization() = default;

    /**
     * 初始化，读配置参数
     * @param yaml_path
     * @param global_map_path
     * @param init_reloc_pose
     */
    bool Init(const std::string& yaml_path, const std::string& global_map_path);

    // 处理lidar消息
    void ProcessLidarMsg(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg);
    void ProcessLivoxLidarMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr laser_msg);

    // 处理IMU消息
    void ProcessIMUMsg(IMUPtr imu);

    // void ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) override;

    // 由外部设置pose，适用于手动重定位
    // NOTE 个人修改（坐标系约定）：入参 q/t 语义 = 【雷达系】在地图下的位姿 T_map_lidar，
    //      与本模块内部、地图、点云约定一致；若你手里是车辆(base_link)系位姿，请改用
    //      SetExternalVehiclePose()（它会做外参换算），不要把车辆位姿直接丢进来。
    void SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    /**
     * 个人修改（2026-10-01，手动初始位姿/重定位）：
     *   入参是【车辆(base_link)系】在地图下的位姿 T_map_vehicle
     *   —— 也就是 RViz "2D Pose Estimate" 给出的位姿（RViz 给的是固定帧下 base_link 的位姿），
     *   以及外部调度/脚本通常能拿到的"车在哪"的位姿。
     *   本函数内部把它换算成定位模块使用的【雷达系】位姿后再重新初始化：
     *     T_map_lidar = ToLidarPose(ToBodyPose(T_map_vehicle), T_imu_lidar_)      // common/vehicle_frame.h
     *   换算后交给 lidar_loc 的 SetInitialPose()（会置 loc_inited_ = false 触发重新初始化），
     *   因此在【运行中任意时刻】调用本函数 = 手动把定位拉回指定位置（重新定位）。
     *   注意：新种子同样要通过初始化门槛 min_init_confidence_ 才会被接受。
     */
    void SetExternalVehiclePose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    // TODO: 其他初始化逻辑

    // TODO: 处理odom消息

    // 结束，保存临时地图
    void Finish();

    // 异步处理函数
    void LidarOdomProcCloud(CloudPtr);
    void LidarLocProcCloud(CloudPtr);

    using TFCallback = std::function<void(const geometry_msgs::msg::TransformStamped& odom)>;
    using LocStateCallback = std::function<void(const std_msgs::msg::Int32& state)>;
    using PointcloudBodyCallback = std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>;
    using PointcloudWorldCallback = std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>;

    void SetTFCallback(TFCallback&& callback);

    // void SetPathCallback(std::function<void(const nav_msgs::msg::Path& path)>&& callback);
    // void SetPointcloudWorldCallback(std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>&& callback);
    // void SetPointcloudBodyCallback(std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>&& callback);
    // void SetLocStateCallback(std::function<void(const std_msgs::msg::Int32& state)>&& callback);
    // void SetHealthDiagNormalCallback(interface::health_diag_normal_callback&& callback);

   private:
    // 模块  ========================================================================================================
    std::mutex global_mutex_;  // 防止处理过程中被重复init
    Options options_;

    // 个人修改："雷达系 → IMU 系"外参（p_imu = R * p_lidar + t），Init() 时从 yaml 的
    //   fasterlio.extrinsic_R/T 读入（与 laser_mapping.cc 读的是同一组参数）。
    //   定位模块输出的位姿是【雷达系】在地图下的位姿，对外发 TF / 送 UI 前用它换算到车辆(base_link)系。
    SE3 T_imu_lidar_;

    // 个人修改：把 LIO 前端给出的【IMU 系】位姿状态换算成【雷达系】位姿状态（其余字段原样保留）。
    //   只在这两个入口做一次（ProcessIMUMsg 的 DR、LidarOdomProcCloud 的 LO），之后
    //   LidarLoc / PGO / 输出平滑器 / 位姿外推器内部就全部统一在【雷达系】下工作，
    //   与地图（雷达系点云拼出的世界系）和输入点云（雷达系）保持一致。
    //   不换算的后果：LidarLoc 的预测初值 (last_abs_pose_ * delta)、CheckLidarOdomValid、
    //   PGO 的 DR/LO 相对约束、平滑器的 pred = pose_queue_.back() * motion 都是
    //   "【雷达系】绝对位姿 × 【IMU 系】相对运动"，在竖装雷达上会整体差一个 90° 外参。
    NavState ToLidarFrameState(const NavState& in) const;

    // 预处理
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess

    // 前端
    std::shared_ptr<LaserMapping> lio_ = nullptr;
    Keyframe::Ptr lio_kf_ = nullptr;

    // ui
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;

    // pose graph
    std::shared_ptr<PGO> pgo_ = nullptr;

    // lidar localization
    std::shared_ptr<LidarLoc> lidar_loc_;

    // TODO async 处理
    sys::AsyncMessageProcess<CloudPtr> lidar_odom_proc_cloud_;  // lidar odom 处理点云
    sys::AsyncMessageProcess<CloudPtr> lidar_loc_proc_cloud_;   // lidar loc 处理点云

    // 结果数据 =====================================================================================================
    LocalizationResult loc_result_;

    // 框架相关
    TFCallback tf_callback_;
    LocStateCallback loc_state_callback_;
    PointcloudBodyCallback pointcloud_body_callback_;
    PointcloudWorldCallback pointcloud_world_callback_;

    // 输入检查
    double last_imu_time_ = 0;
    double last_odom_time_ = 0;
    double last_cloud_time_ = 0;
};
}  // namespace loc

}  // namespace lightning