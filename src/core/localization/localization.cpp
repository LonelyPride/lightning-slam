#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "core/localization/lidar_loc/lidar_loc.h"
#include "core/localization/localization.h"

#include <opencv2/highgui.hpp>

#include "core/localization/pose_graph/pgo.h"
// 个人修改：IMU/雷达 → 车辆(base_link) 的坐标系换算工具
#include "common/vehicle_frame.h"
#include "core/lightning_math.hpp"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"

namespace lightning::loc {

// ！ 构造函数
Localization::Localization(Options options) { options_ = options; }

/**
 * 个人修改：LIO 前端（LaserMapping）的状态位姿是【IMU 系】的 T_w_imu；
 *   而本模块的对外/对下约定是【雷达系】位姿 T_w_lidar（lidar_loc 内部 pose_ = T_map_lidar，
 *   地图是雷达系点云拼出来的，喂进去的 scan 也是雷达系）。两者的关系：
 *       T_w_lidar = T_w_imu * T_imu_lidar
 *   位置只差 R_w_imu * t_imu_lidar（本平台 4 mm，可忽略），姿态差一个 90° 外参。
 *   因此这里在"进模块的门口"换算一次，让 lidar_loc / PGO 内部的相对运动递推、外推、
 *   一致性检查全部自洽。速度/时间戳/标志位等字段与坐标系选择无关，原样保留。
 */
NavState Localization::ToLidarFrameState(const NavState& in) const {
    NavState out = in;
    out.SetPose(vehicle_frame::ToLidarPose(in.GetPose(), T_imu_lidar_));
    return out;
}

// ！初始化函数
bool Localization::Init(const std::string& yaml_path, const std::string& global_map_path) {
    UL lock(global_mutex_);
    if (lidar_loc_ != nullptr) {
        // 若已经启动，则变为初始化
        Finish();
    }

    YAML_IO yaml(yaml_path);
    options_.with_ui_ = yaml.GetValue<bool>("system", "with_ui");

    // lidar odom前端
    LaserMapping::Options opt_lio;
    opt_lio.is_in_slam_mode_ = false;

    lio_ = std::make_shared<LaserMapping>(opt_lio);
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio";
        return false;
    }

    // 个人修改：读取"雷达系 → IMU 系"外参，用的就是上面 lio_->Init() 已经必须存在的那组
    //   yaml 键（fasterlio.extrinsic_R / extrinsic_T），因此这里不会引入新的启动失败点。
    //   定位模块（lidar_loc/PGO）内部算出来的 pose_ 是【雷达系】在地图下的位姿 T_map_lidar，
    //   对外发布 map→base_link 以及送给 UI 显示时，都需要换算到车辆(base_link)系：
    //     T_map_vehicle = T_map_lidar * T_lidar_imu * Mᵀ
    {
        Mat3d R_lidar_imu = math::OrthonormalizeRotation(
            math::MatFromArray<double>(yaml.GetValue<std::vector<double>>("fasterlio", "extrinsic_R")));
        Vec3d t_imu_lidar = math::VecFromArray<double>(yaml.GetValue<std::vector<double>>("fasterlio", "extrinsic_T"));
        T_imu_lidar_ = vehicle_frame::T_imu_lidar(R_lidar_imu, t_imu_lidar);

        LOG(INFO) << "个人修改定位端：lidar->imu 外参 t = " << t_imu_lidar.transpose()
                  << ", R =\n"
                  << R_lidar_imu;
    }

    // 激光定位
    LidarLoc::Options lidar_loc_options;
    lidar_loc_options.update_dynamic_cloud_ = yaml.GetValue<bool>("lidar_loc", "update_dynamic_cloud");
    lidar_loc_options.force_2d_ = yaml.GetValue<bool>("lidar_loc", "force_2d");
    lidar_loc_options.map_option_.enable_dynamic_polygon_ = false;
    lidar_loc_options.map_option_.map_path_ = global_map_path;
    lidar_loc_ = std::make_shared<LidarLoc>(lidar_loc_options);

    if (options_.with_ui_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->SetCurrentScanSize(1);
        ui_->Init();

        // 个人修改：把"雷达系 → IMU 系"外参交给 UI（UI 侧的姿态按车辆系给出，点云仍是雷达系）
        ui_->SetTImuLidar(T_imu_lidar_);

        lidar_loc_->SetUI(ui_);

        // lio_->SetUI(ui_);
    }

    lidar_loc_->Init(yaml_path);

    // pose graph
    pgo_ = std::make_shared<PGO>();
    pgo_->SetDebug(false);

    //  各模块的异步调用
    options_.enable_lidar_loc_skip_ = yaml.GetValue<bool>("system", "enable_lidar_loc_skip");
    options_.enable_lidar_loc_rviz_ = yaml.GetValue<bool>("system", "enable_lidar_loc_rviz");
    options_.lidar_loc_skip_num_ = yaml.GetValue<int>("system", "lidar_loc_skip_num");
    options_.enable_lidar_odom_skip_ = yaml.GetValue<bool>("system", "enable_lidar_odom_skip");
    options_.lidar_odom_skip_num_ = yaml.GetValue<int>("system", "lidar_odom_skip_num");
    options_.loc_on_kf_ = yaml.GetValue<bool>("lidar_loc", "loc_on_kf");

    lidar_odom_proc_cloud_.SetMaxSize(1);
    lidar_loc_proc_cloud_.SetMaxSize(1);

    lidar_odom_proc_cloud_.SetName("激光里程计");
    lidar_loc_proc_cloud_.SetName("激光定位");

    // 允许跳帧
    lidar_loc_proc_cloud_.SetSkipParam(options_.enable_lidar_loc_skip_, options_.lidar_loc_skip_num_);
    lidar_odom_proc_cloud_.SetSkipParam(options_.enable_lidar_odom_skip_, options_.lidar_odom_skip_num_);

    lidar_odom_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarOdomProcCloud(cloud); });
    lidar_loc_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarLocProcCloud(cloud); });

    if (options_.online_mode_) {
        lidar_odom_proc_cloud_.Start();
        lidar_loc_proc_cloud_.Start();
    }

    // TODO: 发布
    pgo_->SetHighFrequencyGlobalOutputHandleFunction([this](const LocalizationResult& res) {
        // if (loc_result_.timestamp_ > 0) {
        //             double loc_fps = 1.0 / (res.timestamp_ - loc_result_.timestamp_);
        //             // LOG_EVERY_N(INFO, 10) << "loc fps: " << loc_fps;
        //         }

        loc_result_ = res;

        // 个人修改：把"雷达→IMU"外参附到结果上，ToGeoMsg()/ToNavState() 会用它把
        //   内部的"雷达系位姿"换算成对外统一的【车辆(base_link)系】姿态
        loc_result_.T_imu_lidar_ = T_imu_lidar_;

        if (tf_callback_ && loc_result_.valid_) {
            tf_callback_(loc_result_.ToGeoMsg());
        }

        if (ui_) {
            ui_->UpdateNavState(loc_result_.ToNavState());
            // 个人修改：UpdateRecentPose 的 pose 用于画车/相机跟随，必须是车辆(base_link)系
            ui_->UpdateRecentPose(vehicle_frame::ToVehiclePoseFromLidar(loc_result_.pose_, T_imu_lidar_));
        }
    });

    // 预处理器
    preprocess_.reset(new PointCloudPreprocess());
    preprocess_->Blind() = yaml.GetValue<double>("fasterlio", "blind");
    preprocess_->TimeScale() = yaml.GetValue<double>("fasterlio", "time_scale");
    int lidar_type = yaml.GetValue<int>("fasterlio", "lidar_type");
    preprocess_->NumScans() = yaml.GetValue<int>("fasterlio", "scan_line");
    preprocess_->PointFilterNum() = yaml.GetValue<int>("fasterlio", "point_filter_num");
    float height_max = yaml.GetValue<float>("roi", "height_max");
    float height_min = yaml.GetValue<float>("roi", "height_min");

    preprocess_->SetHeightROI(height_max, height_min);

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        // 个人修改（2026-10-02）：原日志误写成 "Using OUST 64 Lidar"（复制粘贴），
        //   与 laser_mapping 打印的 "RoboSense" 不一致，排查外参/lidar_type 时极易误导。
        LOG(INFO) << "Using RoboSense Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
    }

    return true;
}

void Localization::ProcessLidarMsg(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    // 串行模式
    CloudPtr laser_cloud(new PointCloudType);
    preprocess_->Process(cloud, laser_cloud);
    laser_cloud->header.stamp = cloud->header.stamp.sec * 1e9 + cloud->header.stamp.nanosec;

    if (options_.online_mode_) {
        lidar_odom_proc_cloud_.AddMessage(laser_cloud);
    } else {
        LidarOdomProcCloud(laser_cloud);
    }
}

void Localization::ProcessLivoxLidarMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    // 串行模式
    CloudPtr laser_cloud(new PointCloudType);
    preprocess_->Process(cloud, laser_cloud);
    laser_cloud->header.stamp = cloud->header.stamp.sec * 1e9 + cloud->header.stamp.nanosec;

    if (options_.online_mode_) {
        lidar_odom_proc_cloud_.AddMessage(laser_cloud);
    } else {
        LidarOdomProcCloud(laser_cloud);
    }
}

void Localization::LidarOdomProcCloud(CloudPtr cloud) {
    if (lio_ == nullptr) {
        return;
    }

    // NOTE: 在NCLT这种数据集中，lio内部是有缓存的，它拿到的点云不一定是最新时刻的点云
    lio_->ProcessPointCloud2(cloud);
    if (!lio_->Run()) {
        return;
    }

    auto lo_state = lio_->GetState();

    // 个人修改：LO 是 LIO 的【IMU 系】位姿，交给 lidar_loc/PGO 之前先换算成【雷达系】(见 ToLidarFrameState)
    const NavState lo_state_lidar = ToLidarFrameState(lo_state);
    lidar_loc_->ProcessLO(lo_state_lidar);
    pgo_->ProcessLidarOdom(lo_state_lidar);

    // LOG(INFO) << "LO pose: " << std::setprecision(12) << lo_state.timestamp_ << " "
    //           << lo_state.GetPose().translation().transpose();

    // 获得lio的关键帧

    auto scan = lio_->GetProjCloud();

    if (options_.loc_on_kf_) {
        auto kf = lio_->GetKeyframe();
        if (kf == lio_kf_) {
            // 关键帧未更新，那就只更新IMU状态

            // auto dr_state = lio_->GetState();
            // lidar_loc_->ProcessDR(dr_state);
            // pgo_->ProcessDR(dr_state);
            return;
        }

        // if (ui_) {
        //     ui_->UpdateKF(kf);
        // }

        lio_kf_ = kf;

        // auto scan = lio_->GetScanUndist();

        if (options_.online_mode_) {
            lidar_loc_proc_cloud_.AddMessage(scan);
        } else {
            LidarLocProcCloud(scan);
        }
    } else {
        // auto scan = cloud;   // 这个cloud应该差一个外参

        if (options_.online_mode_) {
            lidar_loc_proc_cloud_.AddMessage(scan);
        } else {
            LidarLocProcCloud(scan);
        }
    }
}

void Localization::LidarLocProcCloud(CloudPtr scan_undist) {
    lidar_loc_->ProcessCloud(scan_undist);

    auto res = lidar_loc_->GetLocalizationResult();
    pgo_->ProcessLidarLoc(res);

    if (ui_) {
        // Twi with Til, here pose means Twl, thus Til=I
        // 个人修改：UpdateScan 的 pose 语义不变（它能把雷达系点云直接变换到地图，即 T_map_lidar）；
        //   但"画车"的姿态必须另外给【车辆(base_link)系】，否则车会跟着雷达外参一起歪（Airy 竖装约 90°）
        ui_->UpdateScan(scan_undist, res.pose_);
        ui_->UpdateBodyPose(vehicle_frame::ToVehiclePoseFromLidar(res.pose_, T_imu_lidar_));
    }

    if (loc_state_callback_) {
        auto loc_state = std::make_shared<std_msgs::msg::Int32>();
        loc_state->data = static_cast<int>(res.status_);
        LOG(INFO) << "loc_state: " << loc_state->data;
        loc_state_callback_(*loc_state);
    }

    // cv::Mat img(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
    // cv::imshow("img", img);
    // cv::waitKey(0);
}

void Localization::ProcessIMUMsg(IMUPtr imu) {
    UL lock(global_mutex_);

    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    double this_imu_time = imu->timestamp;
    if (last_imu_time_ > 0 && this_imu_time < last_imu_time_) {
        LOG(WARNING) << "IMU 时间异常：" << this_imu_time << ", last: " << last_imu_time_;
    }
    last_imu_time_ = this_imu_time;

    // 里程计处理IMU
    lio_->ProcessIMU(imu);

    // 这里需要 IMU predict，否则没法process DR了
    auto dr_state = lio_->GetIMUState();

    if (!dr_state.pose_is_ok_) {
        return;
    }

    // // 停车判定
    // constexpr auto kThVbrbStill = 0.05;  // 0.08;
    // constexpr auto kThOmegaStill = 0.05;

    // if (dr_state.GetVel().norm() < kThVbrbStill && imu->angular_velocity.norm() < kThOmegaStill) {
    //     dr_state.is_parking_ = true;
    //     dr_state.SetVel(Vec3d::Zero());
    // }

    // 如果没有odm, 用lio替代DR

    // LOG(INFO) << "dr state: " << std::setprecision(12) << dr_state.timestamp_ << " "
    //           << dr_state.GetPose().translation().transpose()
    //           << ", q=" << dr_state.GetPose().unit_quaternion().coeffs().transpose();

    // 个人修改：DR 同样是 LIO 的【IMU 系】位姿，交给 lidar_loc/PGO 之前先换算成【雷达系】
    //   （它们内部要把它与"雷达系绝对位姿"做相对运动递推，见 ToLidarFrameState 的说明）
    const NavState dr_state_lidar = ToLidarFrameState(dr_state);
    lidar_loc_->ProcessDR(dr_state_lidar);
    pgo_->ProcessDR(dr_state_lidar);
}

// void Localization::ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
//     UL lock(global_mutex_);
//
//     if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
//         return;
//     }
//     double this_odom_time = ToSec(odom_msg->header.stamp);
//     if (last_odom_time_ > 0 && this_odom_time < last_odom_time_) {
//         LOG(WARNING) << "Odom Time Abnormal:" << this_odom_time << ", last: " << last_odom_time_;
//     }
//     last_odom_time_ = this_odom_time;
//
//     lio_->ProcessOdometry(odom_msg);
//
//     if (!lio_->GetbOdomHF()) {
//         return;
//     }
//
//     auto dr_state = lio_->GetStateHF(mapping::FasterLioMapping::kHFStateOdomFiltered);
//
//     constexpr auto kThVbrbStill = 0.03;  // 0.08;
//     constexpr auto kThOmegaStill = 0.03;
//     if (dr_state.Getvwi().norm() < kThVbrbStill && dr_state.Getwii().norm() < kThOmegaStill) {
//         dr_state.is_parking_ = true;
//         dr_state.Setvwi(Vec3d::Zero());
//         dr_state.Setwii(Vec3d::Zero());
//     }
//
//     lidar_loc_->ProcessDR(dr_state);
//     pgo_->ProcessDR(dr_state);
// }

void Localization::Finish() {
    lidar_loc_->Finish();
    if (ui_) {
        ui_->Quit();
    }

    lidar_loc_proc_cloud_.Quit();
    lidar_odom_proc_cloud_.Quit();
}

void Localization::SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    UL lock(global_mutex_);
    // 设置外部重定位的pose
    // 个人修改（外参补丁配套说明）：这里的位姿语义 = 【雷达系】在地图下的位姿 T_map_lidar，
    //   与本模块内部及地图/点云约定一致（run_loc_online 传单位阵 = "车辆在地图起点、雷达系与世界系对齐"）。
    //   若外部系统手里是【车辆(base_link)系】位姿，需先换算：
    //     T_map_lidar = ToBodyPoseFromLidar(T_map_vehicle, T_imu_lidar)   // vehicle_frame.h:210
    //   （或直接调用 SetExternalVehiclePose()，2026-10-01 新增，内部完成同一换算）
    if (lidar_loc_) {
        lidar_loc_->SetInitialPose(SE3(q, t));
    }
}

void Localization::SetExternalVehiclePose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    // 个人修改（2026-10-01，手动初始位姿/重定位）：
    //   入参 = 【车辆(base_link)系】在地图下的位姿 T_map_vehicle（RViz "2D Pose Estimate" 语义：
    //   固定帧(map)下 base_link 的位姿）。内部统一使用【雷达系】位姿 T_map_lidar，两者相差两级外参：
    //     T_map_vehicle --ToBodyPose（撤销 IMU→车辆 安装旋转 M = R_vehicle_imu）--> T_map_imu
    //                   --ToLidarPose（乘 雷达→IMU 外参 T_imu_lidar_）----------> T_map_lidar
    //   不换算的后果：本平台雷达/IMU 竖装，二者姿态差约 90°，种子一歪就会被初始化门槛
    //   （lidar_loc.min_init_confidence_）拒掉，表现为"点了 2D Pose Estimate 但定位不跟随"。
    const SE3 T_map_vehicle(q, t);
    const SE3 T_map_lidar = vehicle_frame::ToLidarPose(vehicle_frame::ToBodyPose(T_map_vehicle), T_imu_lidar_);

    LOG(INFO) << "[manual init] T_map_vehicle(base_link): " << T_map_vehicle.translation().transpose() << ", "
              << T_map_vehicle.unit_quaternion().coeffs().transpose();
    LOG(INFO) << "[manual init] -> T_map_lidar (after lidar/imu extrinsics): " << T_map_lidar.translation().transpose()
              << ", " << T_map_lidar.unit_quaternion().coeffs().transpose();

    // 复用既有外部位姿入口：它会锁 global_mutex_ 并调用 lidar_loc_->SetInitialPose()，
    // 后者置 loc_inited_ = false ⇒ 下一帧点云即以该位姿为种子重新初始化。
    SetExternalPose(T_map_lidar.unit_quaternion(), T_map_lidar.translation());
}

void Localization::SetTFCallback(Localization::TFCallback&& callback) { tf_callback_ = callback; }

}  // namespace lightning::loc
