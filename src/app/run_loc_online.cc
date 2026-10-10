//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <algorithm>
#include <sstream>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default_airy96.yaml", "配置文件");

// 个人修改：可选的外部初始位姿，必须是【雷达系】在地图下的位姿 T_map_lidar：
//   "x,y,z,qx,qy,qz,qw"（四元数 x y z w 顺序，内部自动归一化）
// 留空（默认）= 不注入外部位姿，由地图功能点 start 初始化（与 run_loc_offline 一致）。
DEFINE_string(init_pose, "", "可选：外部初始位姿 T_map_lidar \"x,y,z,qx,qy,qz,qw\"；留空=用地图功能点 start");

namespace {

// 解析 "x,y,z,qx,qy,qz,qw"
bool ParseInitPose(const std::string& str, lightning::SE3& pose) {
    std::string text = str;
    std::replace(text.begin(), text.end(), ',', ' ');

    std::istringstream iss(text);
    double v[7] = {0, 0, 0, 0, 0, 0, 1};
    for (double& item : v) {
        if (!(iss >> item)) {
            return false;
        }
    }

    Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
    if (q.norm() < 1e-6) {
        return false;
    }

    pose = lightning::SE3(q.normalized(), lightning::Vec3d(v[0], v[1], v[2]));
    return true;
}

}  // namespace

// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    rclcpp::init(argc, argv);

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    // 个人修改：原来这里固定调用 loc.SetInitPose(SE3())，即把【单位阵】当作车辆在地图起点的位姿。
    //   定位端(lidar_loc)使用的是雷达系在地图下的 T_map_lidar，而"雷达系与世界系重合"并不成立：
    //   本车 lidar→IMU 外参不是单位阵，且 ESKF 初始姿态还含 IMU→车辆安装角，
    //   单位阵因此是一个错误初值（实测配准分值仅 0.66），初始化会被直接带偏且全程无法恢复
    //   （重试逻辑只在未初始化分支内生效），表现就是"定位完全不可用"。
    //   故默认改为：不注入外部位姿，由地图 index.txt 的功能点 start（正确的雷达系位姿）初始化。
    if (FLAGS_init_pose.empty()) {
        loc.Start();
    } else {
        SE3 init_pose;
        if (ParseInitPose(FLAGS_init_pose, init_pose)) {
            LOG(INFO) << "use given init pose (T_map_lidar): " << init_pose.translation().transpose() << ", "
                      << init_pose.unit_quaternion().coeffs().transpose();
            loc.SetInitPose(init_pose);
        } else {
            LOG(ERROR) << "解析 --init_pose 失败，格式应为 \"x,y,z,qx,qy,qz,qw\"，将改用功能点 start：" << FLAGS_init_pose;
            loc.Start();
        }
    }
    // 个人修改（2026-10-01）：手动初始位姿 / 随时重新定位的用法提示。
    //   话题由 yaml 的 common.initial_pose_topic 配置（默认 /initialpose，即 RViz "2D Pose Estimate" 的默认话题），
    //   位姿语义由 common.initial_pose_frame 配置（默认 vehicle = 车辆 base_link 系，与 RViz 一致）。
    LOG(INFO) << "manual init pose: 可在运行中任意时刻用 RViz 的 \"2D Pose Estimate\"（或 ros2 topic pub）向话题 "
              << loc.InitialPoseTopic()
              << " 发送 geometry_msgs/msg/PoseWithCovarianceStamped（frame_id=map，位姿=车辆 base_link 在地图下的位姿）"
                 "，定位模块会立刻以该位姿为种子重新初始化";

    loc.Spin();

    rclcpp::shutdown();

    return 0;
}