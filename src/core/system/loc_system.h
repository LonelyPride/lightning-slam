//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

// 个人修改（2026-10-01）：TF 改为"接上车上已有的 TF 树"后，需要 tf2 的查询接口
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 个人修改（2026-10-01）：把已保存的栅格地图以 nav_msgs/OccupancyGrid 话题形式发布给下游
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <atomic>
#include <string>
#include <thread>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

namespace loc {
class Localization;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;  // 是否发布tf
    };

    // =========================================================================
    //  个人修改（2026-10-01）：把【已保存的栅格地图】以话题形式【持续】发布给下游规划/控制
    //  （nav_msgs/msg/OccupancyGrid）。配置写在 yaml 的 `system.publish_map` 下，
    //  全部是【可选键】，缺键时用这里的默认值（旧 yaml 也能直接跑新二进制）：
    //    enable   开关，默认 true；false => 不建发布者、也不建发布线程
    //    topic    话题名，默认 /localization/map
    //    interval 发布间隔(秒)，默认 1.0（下限保护 0.05 s）
    //    path     栅格地图目录（含 map.pgm + map.yaml）；空 => 用 system.map_path
    //    frame_id 消息 header.frame_id，默认 map
    //
    //  ★为什么放定位端：车运行时下游（规划/控制）一直需要这张地图，而定位进程是长期运行的；
    //    建图进程 run_slam_online 只在建图期间存在，故【建图端不发布】话题，统一由定位端发。
    //  ★数据源：<map_dir>/map.pgm + map.yaml（map_server 格式），即建图端保存出的那一份，
    //    与定位正在用的地图【同源】。定位端没有"正在建的栅格地图"，故没有 source 选项。
    // =========================================================================
    struct MapPublishOptions {
        bool enable_ = true;
        std::string topic_ = "/localization/map";
        double interval_ = 1.0;
        std::string path_ = "";
        std::string frame_id_ = "map";
    };

    // =========================================================================
    //  个人修改（2026-10-01）：定位的 TF 发布方式（把定位接到【车上已有的 TF 树】上）
    //
    //  【问题】车上的 TF 树（本平台实测，见 rosbag 的 /tf、/tf_static）原本是：
    //        odom --(动态)--> base_footprint --(静态)--> base_link --(静态)--> laser_points_* / uwb_*
    //      而本工程旧行为是【直接发 map→base_link】（LocalizationResult::ToGeoMsg），
    //      于是 base_link 同时有 map 与 base_footprint 两个父节点 ⇒ TF 树冲突：
    //      RViz / Nav2 会报 "multiple parents"、或只认其中一条，画面上更会出现 map 与 odom 两棵树错位。
    //
    //  【做法】定位只负责"世界系那一段"，即补 **map→odom** 这一条边：
    //        先查车上已有的 odom→base_link（T_odom_base，tf2 会自动穿过 base_footprint 等中间帧），
    //        再由定位结果 T_map_base 反推：
    //            T_map_odom = T_map_base · (T_odom_base)⁻¹
    //      最终得到一条完整、无冲突的链：  map → odom → base_footprint → base_link → ...
    //
    //  【语义】map→odom 就是里程计系的漂移修正量（车静止时它缓慢变化）；
    //      下游以 map 为 fixed frame 查 base_link，得到的仍然是定位位姿 —— 与旧行为数值等价，
    //      但不会再多出一个父节点。
    //
    //  配置写在 yaml 的 `system.tf` 下，【全部是可选键】，缺键时用这里的默认值（旧 yaml 也能跑）：
    //    enable         开关，默认 true；false => 不建 broadcaster、也不建监听
    //    mode           map_odom（默认）= 发 map→odom（需要车上已有 odom→base_link）；
    //                   map_base        = 老行为，直接发 map→base_link（会与车上 TF 冲突，仅为兼容保留）
    //    map_frame_id   地图帧名，默认 map（仅当定位结果的 header.frame_id 为空时用它兜底；
    //                   正常情况下定位结果自带 "map"，此时以结果里的帧名为准）
    //    odom_frame_id  里程计帧名，默认 odom（必须与车上 TF 一致）
    //    base_frame_id  车体帧名，默认 base_link（仅当定位结果的 child_frame_id 为空时兜底）
    //    lookup_timeout 查 odom→base_link 的最长等待(秒)，默认 0.0 = 只查缓存不等待
    //                   （回调在定位工作线程里，等太久会拖慢定位输出；车上 TF 连续发布，稳态下不用等）
    // =========================================================================
    struct TfOptions {
        bool enable_ = true;
        bool publish_map_to_odom_ = true;
        std::string map_frame_id_ = "map";
        std::string odom_frame_id_ = "odom";
        std::string base_frame_id_ = "base_link";
        double lookup_timeout_ = 0.0;
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    // 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    // 设置初始化位姿
    // NOTE 个人修改（坐标系约定）：本接口传入的必须是【雷达系】在地图下的位姿 T_map_lidar，
    //      即 lidar_loc 内部用来对点云做变换的那个位姿；不要直接传车辆/IMU 位姿，
    //      二者之间还差 lidar→IMU 外参与 IMU→车辆安装角两级外参。
    //      若车辆就停在建图起点，推荐不要调用本函数，改用 Start() 由地图功能点 start 初始化。
    void SetInitPose(const SE3& pose);

    // 个人修改：仅"开始处理数据"，不注入任何外部初始位姿。
    // 初始位姿交给地图 index.txt 里的功能点 start（由 lidar_loc 的 init_with_fp 完成），
    // 与 run_loc_offline 的行为一致，可避免用错坐标系下的种子把初始化带偏。
    void Start();

    // =========================================================================
    //  个人修改（2026-10-01）：手动初始位姿 / 随时重新定位
    //  在线模式下本类订阅一个"手动初始位姿"话题（默认 /initialpose，
    //  即 RViz "2D Pose Estimate" 按钮默认发布的话题，类型
    //  geometry_msgs/msg/PoseWithCovarianceStamped）。收到消息后立即以该位姿为种子
    //  重新初始化（loc_inited_ 清零），于是可以【在运行中任意时刻】把定位校准到鼠标点的地方。
    //  话题名与位姿语义由 yaml 配置：
    //    common.initial_pose_topic : 话题名，默认 "/initialpose"
    //    common.initial_pose_frame : vehicle（默认）= 车辆(base_link)系位姿 T_map_vehicle，
    //                                即 RViz 2D Pose Estimate 的语义；
    //                                lidar = 雷达系位姿 T_map_lidar（与 --init_pose 同语义）。
    //  ★注意：新种子仍需通过 lidar_loc.min_init_confidence_ 门槛才会被接受（否则打印
    //    "init confidence too low ... reject this init candidate" 并按候选顺序继续尝试）。
    // =========================================================================

    // 手动初始位姿（语义 = 车辆 base_link 系在地图下的位姿 T_map_vehicle），内部换算成 T_map_lidar
    void SetInitialPoseFromVehicle(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    // 手动初始位姿（语义 = 雷达系在地图下的位姿 T_map_lidar，与 --init_pose / 功能点 start 一致）
    void SetInitialPoseFromLidar(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    // 当前使用的手动初始位姿话题名（run_loc_online 启动时会打印出来）
    const std::string& InitialPoseTopic() const { return initial_pose_topic_; }

    // 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    // 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    // 实时模式下的spin
    void Spin();

   private:
    Options options_;

    // ---- 个人修改（2026-10-01）：栅格地图话题发布（system.publish_map.*）----
    // 发布线程主体：按 system.publish_map.interval 定时构造并发布 OccupancyGrid
    void MapPublishLoop();

    // 由已保存的 map.pgm + map.yaml 构造消息。读取/解析失败返回 false
    bool LoadGridMsg(const std::string& pgm_path, const std::string& yaml_path, nav_msgs::msg::OccupancyGrid& msg);

    // 栅格地图目录：publish_map.path 非空用它，否则用 yaml 的 system.map_path
    std::string ResolveMapDir() const;

    // ---- 个人修改（2026-10-01）：TF 发布（接上车上已有的 TF 树）----
    // 定位结果的 TF 回调：
    //   mode=map_odom 时，先查车上已有的 odom→base_link，再合成并发布 map→odom；
    //   mode=map_base 时，直接转发 map→base_link（老行为）。
    void PublishTf(const geometry_msgs::msg::TransformStamped& pose);

    // 创建 tf2 监听（Buffer + TransformListener）。监听自带独立线程，不需要额外 spin。
    void StartTfListener();

    TfOptions tf_options_;

    // 查询车上 TF 用（tf2 Buffer 缓存最近 10 s，足够覆盖定位结果与车上 TF 的时标差）
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

    // 日志节流状态：只在定位结果回调线程里读写（单线程），故不加锁
    double tf_warn_stamp_ = -1.0;      // 上一次"查不到 odom→base_link"告警的时间戳
    bool tf_first_pub_logged_ = false;  // 第一帧 map→odom 发布成功时打一次详细日志

    MapPublishOptions map_pub_options_;
    std::string map_dir_;  // yaml 的 system.map_path（publish_map.path 为空时的兜底）

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_ = nullptr;
    std::thread map_pub_thread_;
    std::atomic_bool map_pub_quit_ = false;  // 通知发布线程退出

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    // 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;

    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    // 个人修改（2026-10-01）：手动初始位姿话题（RViz "2D Pose Estimate" → /initialpose）
    std::string initial_pose_topic_ = "/initialpose";
    bool initial_pose_is_vehicle_ = true;  // true=话题位姿语义为车辆(base_link)系 T_map_vehicle；false=雷达系 T_map_lidar

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_ = nullptr;

    // 个人修改：手动初始位姿话题的回调（RViz 2D Pose Estimate / 外部脚本均可）
    void OnInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg);
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
