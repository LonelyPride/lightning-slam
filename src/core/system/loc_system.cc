//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

// 个人修改（2026-10-01）：栅格地图话题发布用（读 map.pgm / map.yaml + 独立线程）
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>

// 个人修改（2026-10-01）：TF 改为发布 map→odom 用（tf2 查询 odom→base_link）
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace lightning {

// 个人修改（2026-10-01）：geometry_msgs 的 TransformStamped → SE3（旋转用四元数，位置直接取）。
//   只在本文件内部使用；四元数非法（范数≈0）时按单位旋转处理，避免 NaN 传下去。
static SE3 TransformToSE3(const geometry_msgs::msg::TransformStamped &tf) {
    Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y,
                         tf.transform.rotation.z);
    if (q.norm() < 1e-6) {
        q = Eigen::Quaterniond::Identity();
    } else {
        q.normalize();
    }

    return SE3(q, Vec3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
}

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    // handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() {
    // 个人修改（2026-10-01）：先让栅格地图发布线程退出并 join，避免线程还在访问已析构的成员
    map_pub_quit_ = true;
    if (map_pub_thread_.joinable()) {
        map_pub_thread_.join();
    }

    if (loc_ != nullptr) {
        loc_->Finish();
    }

    // 个人修改（2026-10-01）：定位线程停掉之后（TF 回调不会再被调用）再拆 TF：
    //   TransformListener 析构时会 cancel 并 join 它自己的监听线程，必须在 node_ 析构之前完成。
    tf_listener_.reset();
    tf_buffer_.reset();
}

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    // 个人修改（2026-10-01）：栅格地图话题发布配置（`system.publish_map.*`）。
    //   这一块【全部是可选键】，因此用 YAML_IO 的 HasValue 探测后再取值
    //   （与下面 common.initial_pose_* 的处理方式一致）：
    //   旧 yaml 没有这一段时保持 MapPublishOptions 里的默认值，新二进制照常启动，不会抛 YAML 异常。
    //   兼容性建议：把【发布间隔】与【话题名】放配置里，下游按需改动不必重新编译。
    map_dir_ = map_path;
    {
        if (yaml.HasValue("system", "publish_map", "enable")) {
            map_pub_options_.enable_ = yaml.GetValue<bool>("system", "publish_map", "enable");
        }

        if (yaml.HasValue("system", "publish_map", "topic")) {
            std::string topic = yaml.GetValue<std::string>("system", "publish_map", "topic");
            if (!topic.empty()) {
                map_pub_options_.topic_ = topic;
            } else {
                LOG(WARNING) << "system.publish_map.topic 为空，按默认值 " << map_pub_options_.topic_ << " 处理";
            }
        }

        if (yaml.HasValue("system", "publish_map", "interval")) {
            map_pub_options_.interval_ = yaml.GetValue<double>("system", "publish_map", "interval");
        }

        if (yaml.HasValue("system", "publish_map", "path")) {
            map_pub_options_.path_ = yaml.GetValue<std::string>("system", "publish_map", "path");
        }

        if (yaml.HasValue("system", "publish_map", "frame_id")) {
            std::string frame = yaml.GetValue<std::string>("system", "publish_map", "frame_id");
            if (!frame.empty()) {
                map_pub_options_.frame_id_ = frame;
            }
        }

        if (map_pub_options_.interval_ < 0.05) {
            LOG(WARNING) << "system.publish_map.interval = " << map_pub_options_.interval_
                         << " 过小，按 0.05 s 处理（原值会让发布线程占满 CPU）";
            map_pub_options_.interval_ = 0.05;
        }
    }

    // 个人修改（2026-10-01）：TF 发布配置（`system.tf.*`）。同样【全部是可选键】，
    //   旧 yaml（没有 system.tf）保持 TfOptions 里的默认值（= 发 map→odom，与车上 TF 不冲突）。
    tf_options_.enable_ = options_.pub_tf_;
    {
        if (yaml.HasValue("system", "tf", "enable")) {
            tf_options_.enable_ = yaml.GetValue<bool>("system", "tf", "enable");
        }

        if (yaml.HasValue("system", "tf", "mode")) {
            std::string mode = yaml.GetValue<std::string>("system", "tf", "mode");
            if (mode == "map_odom") {
                tf_options_.publish_map_to_odom_ = true;
            } else if (mode == "map_base") {
                tf_options_.publish_map_to_odom_ = false;
            } else {
                LOG(WARNING) << "system.tf.mode = \"" << mode << "\" 不是 map_odom / map_base 之一，按 map_odom 处理";
            }
        }

        if (yaml.HasValue("system", "tf", "map_frame_id")) {
            std::string frame = yaml.GetValue<std::string>("system", "tf", "map_frame_id");
            if (!frame.empty()) {
                tf_options_.map_frame_id_ = frame;
            } else {
                LOG(WARNING) << "system.tf.map_frame_id 为空，按默认值 " << tf_options_.map_frame_id_ << " 处理";
            }
        }

        if (yaml.HasValue("system", "tf", "odom_frame_id")) {
            std::string frame = yaml.GetValue<std::string>("system", "tf", "odom_frame_id");
            if (!frame.empty()) {
                tf_options_.odom_frame_id_ = frame;
            } else {
                LOG(WARNING) << "system.tf.odom_frame_id 为空，按默认值 " << tf_options_.odom_frame_id_ << " 处理";
            }
        }

        if (yaml.HasValue("system", "tf", "base_frame_id")) {
            std::string frame = yaml.GetValue<std::string>("system", "tf", "base_frame_id");
            if (!frame.empty()) {
                tf_options_.base_frame_id_ = frame;
            } else {
                LOG(WARNING) << "system.tf.base_frame_id 为空，按默认值 " << tf_options_.base_frame_id_ << " 处理";
            }
        }

        if (yaml.HasValue("system", "tf", "lookup_timeout")) {
            tf_options_.lookup_timeout_ = yaml.GetValue<double>("system", "tf", "lookup_timeout");
            if (tf_options_.lookup_timeout_ < 0.0) {
                LOG(WARNING) << "system.tf.lookup_timeout = " << tf_options_.lookup_timeout_ << " 非法，按 0 处理";
                tf_options_.lookup_timeout_ = 0.0;
            }
        }
    }

    LOG(INFO) << "online mode, creating ros2 node ... ";

    // subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    // 个人修改（2026-10-01）：手动初始位姿话题（RViz "2D Pose Estimate" → /initialpose）。
    //   两个键都是【新增的可选项】，用 YAML_IO::HasValue 探测后取默认值，
    //   这样用旧 yaml（没有这两个键）也能正常启动。
    if (yaml.HasValue("common", "initial_pose_topic")) {
        initial_pose_topic_ = yaml.GetValue<std::string>("common", "initial_pose_topic");
    }
    if (yaml.HasValue("common", "initial_pose_frame")) {
        std::string frame = yaml.GetValue<std::string>("common", "initial_pose_frame");
        if (frame == "vehicle" || frame == "base_link") {
            initial_pose_is_vehicle_ = true;
        } else if (frame == "lidar") {
            initial_pose_is_vehicle_ = false;
        } else {
            LOG(WARNING) << "common.initial_pose_frame = \"" << frame
                         << "\" 不是 vehicle/base_link/lidar 之一，按 vehicle(base_link) 处理";
        }
    }

    if (!initial_pose_topic_.empty()) {
        // 队列深度取 1：手动点选只关心"最后一次点击"，不需要把历史消息全部重放一遍
        initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic_, rclcpp::QoS(1),
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { OnInitialPose(msg); });

        LOG(INFO) << "manual init pose topic: " << initial_pose_topic_
                  << " (semantic: " << (initial_pose_is_vehicle_ ? "T_map_vehicle / base_link, 即 RViz 2D Pose Estimate"
                                                                  : "T_map_lidar, 与 --init_pose 同语义")
                  << ")";
    }

    // 个人修改（2026-10-01）：栅格地图话题发布（system.publish_map.*）。
    //   用【独立线程】而不是 rclcpp timer：构造一帧 OccupancyGrid（几十万~百万格）比较费时，
    //   放在 spin 线程里会挡住 IMU/点云回调（定位会掉帧）。
    if (map_pub_options_.enable_) {
        // 队列深度 1：下游只关心"最新的地图"；地图是持续发送的，丢一帧不影响
        map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_options_.topic_, rclcpp::QoS(1));

        map_pub_thread_ = std::thread([this]() { MapPublishLoop(); });

        LOG(INFO) << "栅格地图发布已开启: topic = " << map_pub_options_.topic_
                  << ", interval = " << map_pub_options_.interval_ << " s, frame_id = " << map_pub_options_.frame_id_
                  << ", map_dir = " << ResolveMapDir();
    } else {
        LOG(INFO) << "栅格地图发布已关闭 (system.publish_map.enable = false)";
    }

    if (tf_options_.enable_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

        // 个人修改（2026-10-01）：mode=map_odom 时需要先知道车上已有的 odom→base_link，
        //   故额外建一个 tf2 监听（只订阅 /tf、/tf_static，不发任何东西）。
        if (tf_options_.publish_map_to_odom_) {
            StartTfListener();
            LOG(INFO) << "tf 发布已开启: " << tf_options_.map_frame_id_ << "→" << tf_options_.odom_frame_id_
                      << "（合成自定位结果 + 车上 TF " << tf_options_.odom_frame_id_ << "→"
                      << tf_options_.base_frame_id_ << "，不再直接发 map→base_link，避免 TF 树冲突）";
        } else {
            LOG(WARNING) << "tf 发布已开启: " << tf_options_.map_frame_id_ << "→" << tf_options_.base_frame_id_
                         << "（system.tf.mode = map_base，老行为；若车上已有 " << tf_options_.odom_frame_id_ << "→"
                         << tf_options_.base_frame_id_ << " 会与 base_link 的父节点冲突）";
        }

        loc_->SetTFCallback([this](const geometry_msgs::msg::TransformStamped &pose) { PublishTf(pose); });
    } else {
        LOG(INFO) << "tf 发布已关闭 (system.tf.enable = false)";
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::Start() {
    // 个人修改：不再注入 SE3() 当初始位姿。
    //   定位端算的是雷达系位姿 T_map_lidar，而"雷达系与世界系重合"并不成立
    //   （差 lidar→IMU 外参、IMU→车辆安装角两级外参），单位阵是错误种子；
    //   交给地图功能点 start（index.txt，正确的雷达系位姿）初始化。
    LOG(INFO) << "start loc without external init pose, use map functional point 'start' instead";
    loc_started_ = true;
}

void LocSystem::SetInitialPoseFromVehicle(const Eigen::Quaterniond &q, const Eigen::Vector3d &t) {
    if (loc_ == nullptr) {
        LOG(ERROR) << "loc is not initialized, ignore manual init pose";
        return;
    }

    LOG(INFO) << "[manual init] 收到车辆(base_link)系位姿，将重新初始化定位：t = " << t.transpose() << ", q(xyz w) = "
              << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w();

    // 手动给了位姿本身就意味着"现在开始/继续定位"（与 SetInitPose 一致），
    // 这样即使启动时忘了 Start()，手动点选也能让数据流进入定位模块。
    loc_started_ = true;
    loc_->SetExternalVehiclePose(q, t);
}

void LocSystem::SetInitialPoseFromLidar(const Eigen::Quaterniond &q, const Eigen::Vector3d &t) {
    if (loc_ == nullptr) {
        LOG(ERROR) << "loc is not initialized, ignore manual init pose";
        return;
    }

    LOG(INFO) << "[manual init] 收到雷达系位姿 T_map_lidar，将重新初始化定位：t = " << t.transpose() << ", q(xyz w) = "
              << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w();

    loc_started_ = true;
    loc_->SetExternalPose(q, t);
}

void LocSystem::OnInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg) {
    const auto &p = msg->pose.pose.position;
    const auto &o = msg->pose.pose.orientation;

    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    if (q.norm() < 1e-6) {
        LOG(ERROR) << "[manual init] 四元数非法(范数≈0)，忽略本条初始位姿: " << o.x << ", " << o.y << ", " << o.z << ", "
                   << o.w;
        return;
    }
    q.normalize();
    const Vec3d t(p.x, p.y, p.z);

    // frame_id 提示：本工程的地图帧名固定为 "map"（见 localization_result.cc ToGeoMsg 发布的 map→base_link），
    //   RViz 的 2D Pose Estimate 给的就是【RViz Fixed Frame】下的 base_link 位姿，所以正常应为 map。
    //   其它帧名只提示、不拒绝（外部脚本可能留空或使用自定义帧名）。
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
        LOG(WARNING) << "[manual init] initial pose 的 frame_id = \"" << msg->header.frame_id
                     << "\"，不是本工程的地图帧 \"map\"；请确认 RViz 的 Fixed Frame 是否被改过";
    }

    if (initial_pose_is_vehicle_) {
        SetInitialPoseFromVehicle(q, t);
    } else {
        SetInitialPoseFromLidar(q, t);
    }
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

// ---------------------------------------------------------------------------
// 个人修改（2026-10-01）：TF 发布 —— 把定位接到【车上已有的 TF 树】上
//
//  车上原有的 TF 树（本平台实测，取自 rosbag 的 /tf 与 /tf_static）：
//      odom --(动态)--> base_footprint --(静态)--> base_link --(静态)--> laser_points_* / uwb_*
//  即 base_link 的父节点是 base_footprint，base_footprint 的父节点是 odom。
//
//  本工程旧行为是【直接发 map→base_link】（LocalizationResult::ToGeoMsg），
//  这会让 base_link 出现【两个父节点】(base_footprint 与 map)：TF 树冲突，
//  下游（RViz / 规划 / Nav2）要么报 "multiple parents"，要么只认其中一条，画面/坐标乱跳。
//
//  正确做法：定位只负责"世界系那一段"，即补 map→odom 这一条边：
//      ① 查车上已有的 odom→base_link（T_odom_base，tf2 会穿过 base_footprint 自动串联）；
//      ② 与定位结果 T_map_base 反推   T_map_odom = T_map_base · (T_odom_base)⁻¹  并发布。
//  最终 TF 树完整且无冲突：  map → odom → base_footprint → base_link → ...
//
//  语义：map→odom 就是里程计系的漂移修正量（车静止时缓慢变化）；
//       下游以 map 为 fixed frame 查 base_link，得到的仍是定位位姿，与旧行为数值等价。
//
//  时标：优先按定位结果的时间戳查（tf2 会插值，精度最高）；
//       查不到（如刚启动、车上 TF 时标滞后）则退化为"取最新可用"；
//       两者都失败时【本帧不发 TF】并给出节流告警 —— 不退回 map→base_link，避免又出现双父节点。
//
//  线程：本函数由定位结果回调调用，运行在定位工作线程（AsyncMessageProcess）里，
//       因此默认不加等待（lookup_timeout=0），以免拖慢定位输出。
// ---------------------------------------------------------------------------
void LocSystem::PublishTf(const geometry_msgs::msg::TransformStamped &pose) {
    if (tf_broadcaster_ == nullptr) {
        return;
    }

    // 老行为（system.tf.mode = map_base）：直接转发定位结果，不做 odom 合成
    if (!tf_options_.publish_map_to_odom_ || tf_buffer_ == nullptr) {
        tf_broadcaster_->sendTransform(pose);
        return;
    }

    const std::string map_frame = pose.header.frame_id.empty() ? tf_options_.map_frame_id_ : pose.header.frame_id;
    const std::string base_frame = pose.child_frame_id.empty() ? tf_options_.base_frame_id_ : pose.child_frame_id;
    const std::string odom_frame = tf_options_.odom_frame_id_;

    // ① 查车上已有的 odom→base_link（中间帧如 base_footprint 由 tf2 自动串联）
    //   时间用 rclcpp::Time + 本节点时钟类型，避免与 Buffer 内部时钟类型不一致（sim time 场景）
    const rclcpp::Time stamp(pose.header.stamp, node_->get_clock()->get_clock_type());

    geometry_msgs::msg::TransformStamped odom_base;
    bool ok = false;
    try {
        odom_base = tf_buffer_->lookupTransform(odom_frame, base_frame, stamp,
                                                rclcpp::Duration::from_seconds(tf_options_.lookup_timeout_));
        ok = true;
    } catch (const tf2::TransformException &) {
        // ② 退化：取最新可用的一帧（例如刚启动时缓存里还没有早于该时刻的样本）
        try {
            odom_base = tf_buffer_->lookupTransform(odom_frame, base_frame, tf2::TimePointZero,
                                                    tf2::durationFromSec(tf_options_.lookup_timeout_));
            ok = true;
        } catch (const tf2::TransformException &) {
            ok = false;
        }
    }

    if (!ok) {
        // 查不到就不发（宁可暂时没有 map→odom，也不要退回 map→base_link 制造双父节点）。
        // 告警按 5 s 节流，避免刷屏。
        const double now = ToSec(pose.header.stamp);
        if (tf_warn_stamp_ < 0 || now - tf_warn_stamp_ > 5.0) {
            tf_warn_stamp_ = now;
            LOG(WARNING) << "[tf] 查不到车上已有的 " << odom_frame << "→" << base_frame << " 变换，本帧不发布 "
                         << map_frame << "→" << odom_frame
                         << "（请确认里程计/底盘节点已发布该 TF；"
                            "如需回到旧行为把 system.tf.mode 设为 map_base）";
        }
        return;
    }

    // ③ 合成 T_map_odom = T_map_base · (T_odom_base)⁻¹
    const SE3 T_map_base = TransformToSE3(pose);
    const SE3 T_map_odom = T_map_base * TransformToSE3(odom_base).inverse();

    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = pose.header.stamp;  // 与定位结果同一时刻，方便下游按时间对齐
    out.header.frame_id = map_frame;
    out.child_frame_id = odom_frame;

    const Vec3d t = T_map_odom.translation();
    const Eigen::Quaterniond q = T_map_odom.so3().unit_quaternion();
    out.transform.translation.x = t.x();
    out.transform.translation.y = t.y();
    out.transform.translation.z = t.z();
    out.transform.rotation.x = q.x();
    out.transform.rotation.y = q.y();
    out.transform.rotation.z = q.z();
    out.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(out);

    if (!tf_first_pub_logged_) {
        tf_first_pub_logged_ = true;
        LOG(INFO) << "[tf] 已开始发布 " << map_frame << "→" << odom_frame << "：由定位结果(" << map_frame << "→"
                  << base_frame << ") 与车上 TF(" << odom_frame << "→" << base_frame
                  << ") 合成，t = " << t.transpose() << "（TF 树: " << map_frame << " → " << odom_frame << " → ... → "
                  << base_frame << "）";
    }
}

// 个人修改（2026-10-01）：创建 tf2 监听，用来查询车上已有的 odom→base_link。
//   说明：TransformListener 默认 spin_thread=true —— 它会把 /tf、/tf_static 的订阅放进
//   一个【独立的 callback group + 独立线程】里 spin（automatically_add_to_executor_with_node=false），
//   因此不会干扰本工程主节点那句单线程 spin(node_)，也不需要在别处再 spin 一次。
//   Buffer 传入 node_ 是为了支持带超时的查询（内部定时器）与 view_frames 调试服务。
void LocSystem::StartTfListener() {
    // 缓存时间用 tf2 默认的 10 s；把 node_ 也传进去，这样带超时的查询（内部定时器）才可用
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock(),
                                                   tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

    LOG(INFO) << "tf 监听已建立（订阅 /tf、/tf_static），用于查询车上已有的 " << tf_options_.odom_frame_id_ << "→"
              << tf_options_.base_frame_id_ << "（中间帧如 base_footprint 会自动串联）";
}

// ---------------------------------------------------------------------------
// 个人修改（2026-10-01）：栅格地图话题发布（system.publish_map.*）
//
//  目的：在线定位进程 run_loc_online 把【已保存的栅格地图】以 nav_msgs/OccupancyGrid
//        按固定间隔【持续】发布给下游规划/控制（默认话题 /localization/map，默认 1 s），
//        下游订阅话题即可，不必自己解析 map.pgm。
//
//  为什么放在定位端：车运行时下游一直需要这张地图，而定位进程是长期运行的；
//        建图进程 run_slam_online 只在建图期间存在，故【建图端不发布】话题，
//        统一由定位端发 —— 发的就是定位正在用的那张 map.pgm / map.yaml（同源）。
//
//  数据源：<map_dir>/map.pgm + map.yaml（map_server 格式，即建图端保存出的那份）。
//        按文件 mtime 判断更新：运行中若重新保存了地图，会自动重载并改发新图，无需重启。
//
//  实现要点：用【独立线程】而不是 rclcpp timer —— 构造一帧消息要遍历整张栅格表
//            （百万格量级），放在 spin 线程里会挡住 IMU/点云回调，导致定位掉帧。
// ---------------------------------------------------------------------------
void LocSystem::MapPublishLoop() {
    LOG(INFO) << "[map pub] 栅格地图发布线程启动: topic = " << map_pub_options_.topic_
              << ", interval = " << map_pub_options_.interval_ << " s, map_dir = " << ResolveMapDir();

    nav_msgs::msg::OccupancyGrid cached_msg;  // 已加载的地图：文件没变就不再读盘
    bool cache_valid = false;
    std::filesystem::file_time_type cached_mtime{};
    bool warned_missing = false;  // "文件不存在"只提示一次，避免刷屏

    while (!map_pub_quit_) {
        const auto tick_begin = std::chrono::steady_clock::now();

        if (map_pub_ != nullptr) {
            const std::string map_dir = ResolveMapDir();
            const std::string pgm_path = map_dir + "/map.pgm";
            const std::string yaml_path = map_dir + "/map.yaml";

            nav_msgs::msg::OccupancyGrid msg;
            bool ok = false;

            std::error_code ec;
            const auto mtime = std::filesystem::last_write_time(pgm_path, ec);
            if (ec) {
                if (!warned_missing) {
                    LOG(WARNING) << "[map pub] 未找到已保存的栅格地图 " << pgm_path
                                 << "（检查 system.map_path / system.publish_map.path 是否指向建图保存的目录）";
                    warned_missing = true;
                }
                cache_valid = false;
            } else {
                warned_missing = false;

                if (cache_valid && mtime == cached_mtime) {
                    // 地图文件没变化：直接发缓存（省掉每秒一遍 1 MB 级别的读盘+解析）
                    msg = cached_msg;
                    ok = true;
                } else {
                    ok = LoadGridMsg(pgm_path, yaml_path, msg);
                    if (ok) {
                        cached_msg = msg;
                        cached_mtime = mtime;
                        cache_valid = true;
                        LOG(INFO) << "[map pub] 已加载栅格地图 " << pgm_path << " (" << msg.info.width << " x "
                                  << msg.info.height << " @ " << msg.info.resolution << " m, origin "
                                  << msg.info.origin.position.x << ", " << msg.info.origin.position.y << ")";
                    } else {
                        cache_valid = false;
                    }
                }
            }

            if (ok) {
                // 时间戳取"发布时刻"（下游据此判断地图的新鲜度）
                msg.header.stamp = node_->now();
                map_pub_->publish(msg);
            }
        }

        // 分片睡眠：既保持"每 interval 秒一帧"，又让 Ctrl-C 的退出请求能在 50 ms 内被响应
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - tick_begin).count();
        double rest = map_pub_options_.interval_ - elapsed;
        while (rest > 0 && !map_pub_quit_) {
            const double step = (rest > 0.05) ? 0.05 : rest;
            std::this_thread::sleep_for(std::chrono::duration<double>(step));
            rest -= step;
        }
    }

    LOG(INFO) << "[map pub] 栅格地图发布线程退出";
}

// 读取已保存的 map.pgm + map.yaml（ROS map_server 格式）并拼成一帧 OccupancyGrid。
//
//  坐标系/行序约定（与 slam.cc:SaveMap 写图时【严格互逆】，改一处必须同步改另一处）：
//    · map.yaml 的 resolution/origin/width/height 直接作为 OccupancyGrid 的元数据；
//      其中 origin 是"地图左下角(即 data[0])在世界系下的坐标"；
//    · 图像第 0 行是【地图顶部】(y 最大)，而 OccupancyGrid 的 data 从【左下角】开始按行排，
//      所以这里读图时用 (height - 1 - y) 行 —— 对应 SaveMap 里写图时的 (height - 1 - y)；
//    · 像素→占据概率（negate=0 的 map_server 语义）：occ = (255 - v) / 255，
//      occ > occupied_thresh ⇒ 100(占据)，occ < free_thresh ⇒ 0(空闲)，其余 ⇒ -1(未知)。
bool LocSystem::LoadGridMsg(const std::string& pgm_path, const std::string& yaml_path,
                            nav_msgs::msg::OccupancyGrid& msg) {
    const cv::Mat img = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        LOG(WARNING) << "[map pub] 无法读取栅格地图图像 " << pgm_path;
        return false;
    }

    // 元数据以同名 map.yaml 为准；文件损坏/缺字段时退回默认值（不因此中断发布）
    double resolution = 0.05;
    double origin_x = 0.0, origin_y = 0.0;
    double occupied_thresh = 0.65, free_thresh = 0.25;
    int negate = 0;
    try {
        const YAML::Node meta = YAML::LoadFile(yaml_path);
        if (meta["resolution"]) {
            resolution = meta["resolution"].as<double>();
        }
        if (meta["negate"]) {
            negate = meta["negate"].as<int>();
        }
        if (meta["occupied_thresh"]) {
            occupied_thresh = meta["occupied_thresh"].as<double>();
        }
        if (meta["free_thresh"]) {
            free_thresh = meta["free_thresh"].as<double>();
        }
        if (meta["origin"] && meta["origin"].size() >= 2) {
            origin_x = meta["origin"][0].as<double>();
            origin_y = meta["origin"][1].as<double>();
        }
    } catch (const std::exception& e) {
        LOG(WARNING) << "[map pub] 解析 " << yaml_path << " 失败(" << e.what()
                     << ")，按 0.05 m/格 与默认阈值处理";
    }

    const int width = img.cols;
    const int height = img.rows;
    if (width <= 0 || height <= 0 || resolution <= 0.0) {
        LOG(WARNING) << "[map pub] 栅格地图尺寸/分辨率异常: " << width << " x " << height << " @ " << resolution;
        return false;
    }

    msg.info.resolution = static_cast<float>(resolution);
    msg.info.width = static_cast<uint32_t>(width);
    msg.info.height = static_cast<uint32_t>(height);
    msg.info.origin.position.x = origin_x;
    msg.info.origin.position.y = origin_y;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), -1);

    for (int y = 0; y < height; ++y) {
        const unsigned char* row = img.ptr<unsigned char>(height - 1 - y);  // 见函数头行序说明
        for (int x = 0; x < width; ++x) {
            const double pixel = static_cast<double>(row[x]) / 255.0;
            const double occ = negate ? pixel : (1.0 - pixel);

            int8_t value = -1;
            if (occ > occupied_thresh) {
                value = 100;
            } else if (occ < free_thresh) {
                value = 0;
            }
            msg.data[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] = value;
        }
    }

    msg.header.frame_id = map_pub_options_.frame_id_;
    return true;
}

// 栅格地图目录：配置了 system.publish_map.path 就用它，否则用 yaml 的 system.map_path
//   （定位端的地图目录，里面就是建图保存出的 map.pgm / map.yaml / global.pcd）
std::string LocSystem::ResolveMapDir() const {
    std::string dir = map_pub_options_.path_;
    if (dir.empty()) {
        dir = map_dir_;
    }

    // 都没配时退回本工程默认地图目录，并在日志里能看出来是从哪来的
    if (dir.empty()) {
        dir = "./data/new_map/";
    }

    // 去掉结尾多余的 '/'，避免拼出 "...//map.pgm"（filesystem 能处理，但日志和报错不好看）
    while (dir.size() > 1 && dir.back() == '/') {
        dir.pop_back();
    }

    return dir;
}

}  // namespace lightning