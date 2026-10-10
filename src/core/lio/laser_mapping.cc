#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "common/options.h"
// 个人修改：IMU 姿态 → 车辆(base_link)姿态 的换算工具（详见 common/vehicle_frame.h 头部说明）
#include "common/pose_rpy.h"
#include "common/vehicle_frame.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

namespace lightning {

bool LaserMapping::Init(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, ESKF::state_dim_, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    // 个人修改（2026-10-02，方案C）：接通"重力作为加计观测"（原 ESKF 已预留接口但从未注册/调用）。
    //   它是姿态的【绝对竖直参考】：低动态时把"加计测到的比力方向 = 竖直向上"作为软观测并入 ESKF，
    //   防止 roll/pitch 被 LiDAR 更新（尤其是几何退化场景）冻住后再也无法纠回。
    eskf_options.acc_as_gravity_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) {
        AccAsGravityObsModel(s, obs);
    };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();

        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        // 个人修改（2026-10-02 第5次修复）：可选键 fasterlio.ground_normal_lidar = 雷达系里的
        //   【实测路面法向】（单位向量，缺省 (0,1,0) = 旧行为）。它只用于启动时的 up 方向自检
        //   参照（见 TryInitVehicleFrameFromGravity 与 laser_mapping.h 的说明）：
        //   本平台雷达相对重力倾斜安装，雷达 +y 与实测路面法向差 10.59°，只有用实测法向作参照，
        //   自检才等价于"地图地面是否水平"。
        if (yaml["fasterlio"]["ground_normal_lidar"].IsDefined() &&
            !yaml["fasterlio"]["ground_normal_lidar"].IsNull()) {
            const auto n = yaml["fasterlio"]["ground_normal_lidar"].as<std::vector<double>>();
            if (n.size() == 3) {
                const double nn = Vec3d(n[0], n[1], n[2]).norm();
                if (nn > 1e-6) {
                    ground_normal_lidar_ = Vec3d(n[0], n[1], n[2]) / nn;
                } else {
                    LOG(WARNING) << "个人修改（第5次修复）：fasterlio.ground_normal_lidar 近似零向量，已退回 (0,1,0)";
                }
            } else {
                LOG(WARNING) << "个人修改（第5次修复）：fasterlio.ground_normal_lidar 需要 3 个数，实际 "
                             << n.size() << " 个，已退回 (0,1,0)";
            }
        }

        // 个人修改（2026-10-09 第6次修复，方案D）：可选键 fasterlio.imu_acc_up_sign = 加计比力【方向符号】
        //   （缺省 +1 = 旧行为，不影响其它平台）。本平台（狗）驱动把比力方向反了号，必须设 -1：
        //   同一 bag 两条独立实测都指向 -1（地面法向 ⇒ angle=179.23°；tf_static 链 ⇒ 179.52°），
        //   详见 imu_processing.hpp::SetAccUpSign() 注释。这里只做合法性检查，真正的应用在下面 SetExtrinsic 旁。
        if (yaml["fasterlio"]["imu_acc_up_sign"].IsDefined() && !yaml["fasterlio"]["imu_acc_up_sign"].IsNull()) {
            const double s = yaml["fasterlio"]["imu_acc_up_sign"].as<double>();
            if (s == 1.0 || s == -1.0) {
                imu_acc_up_sign_ = s;
            } else {
                LOG(WARNING) << "个人修改（第6次修复）：fasterlio.imu_acc_up_sign 只接受 +1/-1，实际 " << s
                             << " ⇒ 已退回 +1（标准加计约定）";
            }
        }

        ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

        float height_max = yaml["roi"]["height_max"].as<float>();
        float height_min = yaml["roi"]["height_min"].as<float>();

        preprocess_->SetHeightROI(height_max, height_min);

        options_.kf_dis_th_ = yaml["fasterlio"]["kf_dis_th"].as<double>();
        options_.kf_angle_th_ = yaml["fasterlio"]["kf_angle_th"].as<double>() * M_PI / 180.0;
        options_.enable_icp_part_ = yaml["fasterlio"]["enable_icp_part"].as<bool>();
        options_.min_pts = yaml["fasterlio"]["min_pts"].as<int>();
        options_.plane_icp_weight_ = yaml["fasterlio"]["plane_icp_weight"].as<float>();

        bool use_imu_filter = yaml["fasterlio"]["imu_filter"].as<bool>();
        p_imu_->SetUseIMUFilter(use_imu_filter);
        options_.proj_kfs_ = yaml["fasterlio"]["proj_kfs"].as<bool>();

    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    // 个人修改（方案A+）：IMU → 车辆(base_link) 安装旋转（车体安装姿态）的读取。
    //   三项都可以不写：缺省 = identity = 不做换算（等价于加方案A之前的输出）。均为可选键，
    //   放在 common 段，与上面的必需参数分开，缺了也不会触发 "bad conversion" 起不来。
    //     common.vehicle_imu_mount        : auto / identity / x_down / x_up
    //     common.vehicle_imu_forward_axis : auto 模式下"车头"对应的 IMU 轴（x/y/z，可带负号，默认 z）
    //     common.vehicle_imu_R            : 行主序 9 个数，直接给出 M（优先级最高，可描述任意安装）
    {
        const auto has_key = [](const YAML::Node &n) { return n.IsDefined() && !n.IsNull(); };

        if (has_key(yaml["common"]["vehicle_imu_R"])) {
            std::vector<double> v9;
            try {
                v9 = yaml["common"]["vehicle_imu_R"].as<std::vector<double>>();
            } catch (const std::exception &e) {
                LOG(ERROR) << "个人修改（方案A+）common.vehicle_imu_R 解析失败: " << e.what();
            }
            if (v9.size() == 9) {
                const Mat3d R = math::MatFromArray<double>(v9);
                if (vehicle_frame::SetVehicleImuRotation(R)) {
                    vehicle_imu_ready_ = true;
                    LOG(INFO) << "个人修改（方案A+）车辆(base_link)系安装旋转取 common.vehicle_imu_R:\n" << R;
                } else {
                    LOG(ERROR) << "个人修改（方案A+）common.vehicle_imu_R 不是合法旋转矩阵（需正交且 det=+1），"
                                  "本次不做 IMU→车辆 换算";
                }
            } else if (!v9.empty()) {
                LOG(ERROR) << "个人修改（方案A+）common.vehicle_imu_R 需要 9 个数（行主序），当前 " << v9.size() << " 个，已忽略";
            }
        }

        if (!vehicle_imu_ready_) {
            std::string mount;
            if (has_key(yaml["common"]["vehicle_imu_mount"])) {
                mount = yaml["common"]["vehicle_imu_mount"].as<std::string>();
            }
            if (has_key(yaml["common"]["vehicle_imu_forward_axis"])) {
                vehicle_imu_forward_axis_ = yaml["common"]["vehicle_imu_forward_axis"].as<std::string>();
            }

            std::string key;  // 去掉空白并统一小写，便于判断 auto
            for (char c : mount) {
                if (!std::isspace(static_cast<unsigned char>(c))) {
                    key.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
                }
            }

            if (key.empty()) {
                LOG(WARNING) << "个人修改（方案A+）未配置 common.vehicle_imu_mount，IMU→车辆(base_link) 换算按单位阵处理"
                                "（与加方案A之前一致）。若本平台 IMU 是竖装(如 Airy 96)，建议在 yaml 里加：\n"
                                "  common:\n"
                                "    vehicle_imu_mount: auto\n"
                                "    vehicle_imu_forward_axis: z";
            } else if (key == "auto") {
                vehicle_imu_auto_ = true;
                LOG(INFO) << "个人修改（方案A+）车辆(base_link)系安装旋转: auto（等 IMU 静止初始化后按重力方向自动对齐，"
                          << "车头轴 = " << vehicle_imu_forward_axis_ << "）";
            } else {
                Mat3d R;
                if (vehicle_frame::VehicleImuRotationFromName(key, &R) && vehicle_frame::SetVehicleImuRotation(R)) {
                    vehicle_imu_ready_ = true;
                    LOG(INFO) << "个人修改（方案A+）车辆(base_link)系安装旋转取 common.vehicle_imu_mount=" << key << ":\n" << R;
                } else {
                    LOG(ERROR) << "个人修改（方案A+）common.vehicle_imu_mount 取值非法: " << mount
                               << "（应为 auto / identity / x_down / x_up，或改用 common.vehicle_imu_R），本次不做换算";
                }
            }
        }
    }

    // =============================================================================================
    //  个人修改（2026-10-02，方案C：姿态/重力根治）—— 新增参数（全部为【可选键】，缺键用代码默认值）
    //    · fasterlio.imu_init_*           : IMU 静止初始化窗口判据（|a|-g 容差、角速度容差、抖动容差、
    //                                       最短/最长窗口）—— 解决"上车即开动导致重力对齐被污染"
    //    · fasterlio.gravity_obs_*        : 姿态的重力（竖直）软观测参数
    //    · fasterlio.world_frame_realign_* : 世界系重力再对齐（把倾斜的地图/轨迹整体刚性转回竖直）
    //  读取位置：本函数；调用位置：Run()（重力观测/再对齐）、ImuProcess::Process()（初始化判据）
    // =============================================================================================
    {
        const auto has_key = [](const YAML::Node &n) { return n.IsDefined() && !n.IsNull(); };
        const auto get_num = [&](const char *key, double fallback) {
            return has_key(yaml["fasterlio"][key]) ? yaml["fasterlio"][key].as<double>() : fallback;
        };
        const auto get_int = [&](const char *key, int fallback) {
            return has_key(yaml["fasterlio"][key]) ? yaml["fasterlio"][key].as<int>() : fallback;
        };
        const auto get_bool = [&](const char *key, bool fallback) {
            return has_key(yaml["fasterlio"][key]) ? yaml["fasterlio"][key].as<bool>() : fallback;
        };

        const double init_acc_tol = get_num("imu_init_acc_tol", 0.15);
        const double init_gyr_tol = get_num("imu_init_gyr_tol", 0.10);
        const double init_std_rel_tol = get_num("imu_init_acc_std_rel_tol", 0.05);
        const double init_min_dur = get_num("imu_init_min_duration_s", 0.5);
        const double init_max_dur = get_num("imu_init_max_duration_s", 5.0);
        const int init_max_wait = get_int("imu_init_max_wait_count", 100);
        p_imu_->SetInitStaticParams(init_acc_tol, init_gyr_tol, init_std_rel_tol, init_min_dur, init_max_dur,
                                    init_max_wait);

        gravity_obs_enable_ = get_bool("gravity_obs_enable", true);
        gravity_obs_sigma_deg_ = get_num("gravity_obs_sigma_deg", 1.5);
        gravity_obs_acc_tol_ = get_num("gravity_obs_acc_tol", 0.15);
        gravity_obs_gyr_tol_ = get_num("gravity_obs_gyr_tol", 0.15);
        // 个人修改（方案C，2026-10-02 加固）：比力【方向】散布门限（度），见 laser_mapping.h 的说明
        gravity_obs_acc_dir_tol_deg_ = get_num("gravity_obs_acc_dir_tol_deg", 3.0);
        // 个人修改（方案C，2026-10-02 第2次加固）：ba 一致性告警阈值（度），见 laser_mapping.h 的说明
        gravity_obs_ba_warn_deg_ = get_num("gravity_obs_ba_warn_deg", 2.0);
        // 个人修改（方案C，2026-10-02 第3次加固）：【平动加速度】门限（m/s²）+ 速度差分基线（s），
        // 见 laser_mapping.h 里"平动加速度门限"的说明（加计自己的三个门限看不出恒定的加/减速）
        gravity_obs_motion_acc_tol_ = get_num("gravity_obs_motion_acc_tol", 0.35);
        gravity_obs_motion_window_s_ = get_num("gravity_obs_motion_window_s", 1.0);
        // 个人修改（方案C，2026-10-02 第3次加固）：世界系再对齐只采信【静止】样本（m/s），
        // 见 laser_mapping.h 里"静止门限"的说明（挡运动中的瞬时冲击假重力）
        gravity_obs_max_speed_mps_ = get_num("gravity_obs_max_speed_mps", 0.05);
        gravity_obs_every_n_scan_ = get_int("gravity_obs_every_n_scan", 1);
        gravity_obs_sigma_scale_ = get_num("gravity_obs_sigma_scale", 1.0);
        gravity_obs_prior_inflation_deg_ = get_num("gravity_obs_prior_inflation_deg", 0.5);
        // 个人修改（方案C，2026-10-02 第4次修复①）：【行驶中的竖直锚】参数（见 laser_mapping.h）
        gravity_obs_motion_enable_ = get_bool("gravity_obs_motion_enable", true);
        gravity_obs_motion_sigma_deg_ = get_num("gravity_obs_motion_sigma_deg", 3.0);
        gravity_obs_motion_acc_dir_tol_deg_ = get_num("gravity_obs_motion_acc_dir_tol_deg", 10.0);
        gravity_obs_motion_min_speed_mps_ = get_num("gravity_obs_motion_min_speed_mps", 0.1);

        world_frame_realign_enable_ = get_bool("world_frame_realign", true);
        world_frame_realign_min_deg_ = get_num("world_frame_realign_min_deg", 0.7);
        world_frame_realign_max_deg_ = get_num("world_frame_realign_max_deg", 30.0);
        world_frame_realign_max_kf_id_ = get_int("world_frame_realign_max_kf", 40);
        world_frame_realign_max_time_s_ = get_num("world_frame_realign_max_time_s", 15.0);
        world_frame_realign_min_samples_ = get_int("world_frame_realign_min_samples", 20);

        LOG(INFO) << "[方案C] imu init: acc_tol=" << init_acc_tol << " m/s^2, gyr_tol=" << init_gyr_tol
                  << " rad/s, std_rel_tol=" << init_std_rel_tol << ", window=[" << init_min_dur << ", " << init_max_dur
                  << "]s, max_wait=" << init_max_wait;
        LOG(INFO) << "[方案C] gravity obs: enable=" << gravity_obs_enable_ << ", sigma=" << gravity_obs_sigma_deg_
                  << " deg, acc_tol=" << gravity_obs_acc_tol_ << ", gyr_tol=" << gravity_obs_gyr_tol_
                  << " rad/s, acc_dir_tol=" << gravity_obs_acc_dir_tol_deg_
                  << " deg, ba_warn=" << gravity_obs_ba_warn_deg_
                  << " deg, motion_acc_tol=" << gravity_obs_motion_acc_tol_ << " m/s^2 (窗口 "
                  << gravity_obs_motion_window_s_ << " s，来自 LIO 速度差分)"
                  << ", 再对齐静止门限=" << gravity_obs_max_speed_mps_ << " m/s"
                  << ", every_n_scan=" << gravity_obs_every_n_scan_;
        // 个人修改（方案C，2026-10-02 第4次修复①）：行驶中的竖直锚（见 laser_mapping.h 里"第4次修复①"）
        LOG(INFO) << "[方案C] gravity obs(行驶中竖直锚): enable=" << gravity_obs_motion_enable_
                  << ", sigma=" << gravity_obs_motion_sigma_deg_ << " deg, 方向散布门限="
                  << gravity_obs_motion_acc_dir_tol_deg_ << " deg, 判定\"在动\"的 |v| 门限="
                  << gravity_obs_motion_min_speed_mps_ << " m/s（平动加速度门限 " << gravity_obs_motion_acc_tol_
                  << " m/s² 必须同时满足且有效）";
        LOG(INFO) << "[方案C] world frame realign: enable=" << world_frame_realign_enable_ << ", tilt range=["
                  << world_frame_realign_min_deg_ << ", " << world_frame_realign_max_deg_ << "] deg, max_kf="
                  << world_frame_realign_max_kf_id_ << ", max_time=" << world_frame_realign_max_time_s_ << "s";
    }

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
        LOG(INFO) << "Using RoboSense Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    offset_t_lidar_fixed_ = math::VecFromArray<double>(extrinT_);
    // 个人修改（2026-10-02，第 4 次修复③ 配套）：先把 yaml 读进来的 extrinsic_R 正交化。
    //   原因见 lightning_math.hpp::OrthonormalizeRotation() 的注释：yaml 里手写的 9 个数带
    //   ~1e-9 非正交误差，会被 Sophus 的 SO3(R) 前置检查直接 abort（实测本平台新外参 7.0e-9）。
    {
        const Mat3d R_raw = math::MatFromArray<double>(extrinR_);
        offset_R_lidar_fixed_ = math::OrthonormalizeRotation(R_raw);
        const double ortho_err = (R_raw * R_raw.transpose() - Mat3d::Identity()).norm();
        if (ortho_err > 1e-9) {
            LOG(WARNING) << "个人修改：fasterlio.extrinsic_R 不是严格正交（|R*Rᵀ-I| = " << ortho_err
                         << "，det = " << R_raw.determinant() << "），已按四元数归一化投影到最近旋转矩阵"
                         << "（否则 Sophus 的 SO3(R) 会直接 abort，见 lightning_math.hpp 注释）";
        }
    }

    p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
    // 个人修改（2026-10-09 第6次修复，方案D）：加计比力方向符号（见 imu_processing.hpp::SetAccUpSign 注释）。
    //   必须在 IMU 初始化之前设定：IMUInit 用它定向 mean_acc_（⇒ 世界系 z 才是物理向上），
    //   UndistortPcl 用它定向每帧比力（⇒ 预测项 v̇ = R·f + g 才不会被算成 -2g）。
    p_imu_->SetAccUpSign(imu_acc_up_sign_);
    LOG(INFO) << "个人修改（第6次修复）加计比力方向符号 fasterlio.imu_acc_up_sign = " << p_imu_->GetAccUpSign()
              << "（+1 = 静止读数朝上[标准]；-1 = 该平台驱动反号[本平台实测]）";
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

/**
 * 个人修改（方案A+）：auto 模式下用 IMU 静止初始化窗口的重力方向确定 IMU→车辆 安装旋转。
 *
 *   原理：IMU 静止时加速度计测到的比力方向就是"上"（imu_processing.hpp::IMUInit() 内部
 *        用同一个 mean_acc_ 做世界系重力对齐），因此不需要事先知道安装倾角；
 *        车头方向取 common.vehicle_imu_forward_axis 指定轴在水平面内的投影。
 *        对"IMU 有垫片/支架倾斜、换批次、雷达同时改水平或倾斜安装"等情况都能自动适应，
 *        因为雷达安装只影响 fasterlio.extrinsic_R/T，与车体安装无关。
 *   只在一帧上生效一次，之后 vehicle_imu_ready_ 置位，不再改动。
 */
void LaserMapping::TryInitVehicleFrameFromGravity() {
    if (!vehicle_imu_auto_ || vehicle_imu_ready_ || !p_imu_->IsIMUInited()) {
        return;
    }

    const Vec3d mean_acc = p_imu_->GetMeanAcc();  // 静止比力（IMU 系），单位任意
    Mat3d R;
    std::string err;
    if (!vehicle_frame::VehicleImuRotationFromGravity(mean_acc, vehicle_imu_forward_axis_, &R, &err)) {
        LOG(ERROR) << "个人修改（方案A+）按重力自动对齐车辆(base_link)系失败: " << err
                   << "；本次不做 IMU→车辆 换算（可修改 common.vehicle_imu_mount / vehicle_imu_R 后重启）";
        vehicle_imu_ready_ = true;  // 置位以避免每帧重试刷日志；换算退化为 identity
        return;
    }

    vehicle_frame::SetVehicleImuRotation(R);
    vehicle_imu_ready_ = true;
    LOG(INFO) << "个人修改（方案A+）车辆(base_link)系安装旋转已按重力自动对齐: mean_acc(IMU 系) = " << mean_acc.transpose()
              << " (|a| = " << mean_acc.norm() << "), 车头轴 = " << vehicle_imu_forward_axis_ << "\nR_vehicle_imu =\n"
              << R;

    // 个人修改（方案C，2026-10-02 第4次修复③）：【外参 up 方向自检】。
    //   背景：fasterlio.extrinsic_R 描述的是"雷达系→IMU 系"的【单元内部】固定关系（同一台 Airy 里
    //     IMU 与雷达是刚体），其中"地面法向（雷达 +y，本 bag 实测地面 ⊥ 雷达 +y 在 0.24~0.40° 内）"
    //     应当映射到"IMU 系里的重力反方向"。若配置把这一步写错，就会出现：
    //       雷达看到的地面（倾斜坐标）≠ 世界系 z（由加计重力定义）⇒ 地图地面比世界 z 倾斜同角度
    //       ⇒ z 沿前进方向按 tan(该角) 漂移（本平台历史差 11.33° ⇒ 理论 0.20 m/m）。
    //   ⇒ 这里在启动瞬间（IMU 静止窗口均值可得）就把这个角算出来并打印：
    //       > 0.4°：WARNING 提示外参"单元内部朝向"与实物不符（改 fasterlio.extrinsic_R 后重跑复核）；
    //       ≤ 0.4°：INFO 通过。
    //   注意：本自检只读、不修改任何配置/状态（外参仍由 yaml 决定，便于复现与对比）。
    if (mean_acc.allFinite() && mean_acc.norm() > 1e-6) {
        const Vec3d up_meas = mean_acc.normalized();                              // IMU 系实测"上"（静止窗口均值）
        // 个人修改（2026-10-02 第5次修复）：参照向量改为【雷达系实测路面法向】ground_normal_lidar_。
        //   第4次修复用的是硬编码的雷达 +y（= (0,1,0)）：本平台雷达单元相对重力是【倾斜安装】的
        //   （+y 与实测路面法向差 10.590°）⇒ 用 +y 当参照等价于把"路面"拧 10.59° ⇒ 地图地面与墙
        //   整体刚性倾斜、z 沿前进方向按 tan(10.6°)=0.187 m/m 漂移（4 次修复后 val_4th_b 实测地面
        //   倾角正是 10.602°）。正确判据：angle(R·n_l, a0) —— R 把"看到的路面法向"映射到"加计竖直"
        //   时，地图地面才与世界系 z 一致。
        const Vec3d up_cfg = (offset_R_lidar_fixed_ * ground_normal_lidar_).normalized();  // 标定参照下的"上"
        const Vec3d up_cfg_y = (offset_R_lidar_fixed_ * Vec3d(0, 1, 0)).normalized();     // 诊断：R·(+y)
        const double cos_up = std::min(1.0, std::max(-1.0, up_cfg.dot(up_meas)));
        const double cos_up_y = std::min(1.0, std::max(-1.0, up_cfg_y.dot(up_meas)));
        extrinsic_up_check_deg_ = std::acos(cos_up) * 180.0 / M_PI;
        const double mount_tilt_deg = std::acos(cos_up_y) * 180.0 / M_PI;  // = 单元相对重力的安装倾角
        // 个人修改（2026-10-09 第6次修复，方案D）：判据分级。
        //   背景：修订前用 0.4° 单一阈值，实测【真实地面与重力的夹角】常有零点几度~数度（地面本身有坡度、
        //   法向拟合偏差、狗站得不平），会被一律报成"外参写错、请重算 extrinsic_R"，属误报。
        //   现在：> 3° 才判为外参/参照向量写错（历史两次真错分别是 10.59°、21.93°、137.9°，都能抓到）；
        //   0.4°~3° 打印 INFO 说明这是"地面/拟合残差"，不是外参错误。
        constexpr double kUpCheckTolDeg = 0.4;    // ≤ 该值视为"精确通过"
        constexpr double kUpCheckSlopeDeg = 3.0;  // > 该值才判为外参/参照向量错误
        if (extrinsic_up_check_deg_ > kUpCheckSlopeDeg) {
            LOG(WARNING) << "个人修改（第5次修复）【外参 up 方向自检(路面法向参照)】不一致：fasterlio.extrinsic_R 把"
                         << "雷达系路面法向 " << ground_normal_lidar_.transpose() << " 映射到 IMU 的 "
                         << up_cfg.transpose() << "，而 IMU 静止窗口实测竖直 = " << up_meas.transpose()
                         << "，相差 " << extrinsic_up_check_deg_ << " deg ⇒ 雷达看到的地面与世界系 z 差同角度"
                         << "（地图地面会倾斜、z 会沿前进方向按 tan 漂移）。请检查 fasterlio.extrinsic_R 与"
                         << " fasterlio.ground_normal_lidar 是否都是本平台的实测值（标定方程 R·n_l = a0）；"
                         << "若两者的写法与平台不符（例如照抄了其它车的 extrinsic_R），请按本平台重标。"
                         << "★不要改按雷达 +y：本平台 R·(+y) 与实测竖直差 " << mount_tilt_deg
                         << " deg，那是【单元安装倾角 / 雷达安装姿态】，属正常现象。";
        } else if (extrinsic_up_check_deg_ > kUpCheckTolDeg) {
            LOG(INFO) << "个人修改（第6次修复）【外参 up 方向自检(路面法向参照)】通过（小残差）：R 把雷达系路面法向 "
                      << ground_normal_lidar_.transpose() << " 映射为 " << up_cfg.transpose() << "，与实测竖直 "
                      << up_meas.transpose() << " 相差 " << extrinsic_up_check_deg_ << " deg（≤ " << kUpCheckSlopeDeg
                      << "° ⇒ 不是外参错误）。该零点几度~数度的残差是【地面本身与重力的夹角（坡度）/ 平面拟合偏差 /"
                      << " 建标定时车没站平】—— 地面法向取自点云、竖直取自加计，两者物理上不必严格重合；"
                      << "若想要 ≤0.4° 的\"精确通过\"，请在【平坦地面】上静止重测 ground_normal_lidar。"
                      << "（诊断：R·(+y) 与该竖直差 " << mount_tilt_deg << " deg = 单元安装倾角，正常）";
        } else {
            LOG(INFO) << "个人修改（第5次修复）【外参 up 方向自检(路面法向参照)】通过：R 把雷达系路面法向 "
                      << ground_normal_lidar_.transpose() << " 映射为 " << up_cfg.transpose()
                      << "，与实测竖直 " << up_meas.transpose() << " 相差 " << extrinsic_up_check_deg_
                      << " deg（≤ 0.4°）⇒ 地图地面与世界系 z 一致，不存在\"地面倾斜⇒z 漂移\"这一项"
                      << "（诊断：R·(+y) 与该竖直差 " << mount_tilt_deg << " deg = 单元安装倾角，正常）";
        }
    }
    return;
}

// =================================================================================================
//  个人修改（2026-10-02，方案C：姿态/重力根治）—— 姿态的绝对竖直参考 + 世界系自检/再对齐
//
//  【背景：为什么"世界系倾斜"会让 z 轴持续下漂】
//    ESKF 的姿态 R_w_imu 是"世界系里看 IMU"，世界系本身由【初始化那一刻的对齐结果】定义。
//    若初始化对齐被运动加速度污染（或运行中被 LiDAR 拖歪），世界系的 z 轴就不再指向重力方向，
//    于是"车在真实水平面上直行"这一段运动，在世界系里就带上了竖直分量：
//      · 轨迹整体刚性倾斜（本 bag 实测 24.32°，Kabsch 残差 rms 0.11 m）；
//      · 地图地面在路径区 3.7 m 内下降 1.12 m（斜率 0.404 ≈ 22°）；
//      · 车体实际水平（雷达离地 0.68 m 恒定），却表现为"z 一直往下漂"。
//    原代码没有任何绝对竖直参考：陀螺积分没有基准、LiDAR 点面观测在几何退化时会放任 roll/pitch，
//    而 ESKF 里既没有加计零偏也没有重力状态，一旦 P 收缩就再也纠不回来。
//
//  【本组函数给出的三级防线】
//    ① ImuProcess::IMUInit()：静止窗口判据 + 幅值一致性（初始化对齐正确）—— 见 imu_processing.hpp
//    ② AccAsGravityObsModel(): 低动态时把"加计方向 = 竖直向上"作为【软观测】并入 ESKF
//       ⇒ 姿态永远有一个绝对竖直参考，不会再被 LiDAR 拖歪（这是"根治"，不是打补丁）
//    ③ TryRealignWorldFrameToGravity(): 万一世界系已经歪了（历史地图/历史 bag），
//       在建图早期把位姿 + 关键帧 + 局部地图整体【刚性旋转】回竖直 ⇒ 已倾斜的地图一次性纠正
//       （刚性旋转 = 只换世界系基准，不引入任何形变）
// =================================================================================================

/**
 * 个人修改（方案C）：重力方向（竖直）作为姿态观测。
 *
 * 观测模型（与 NavState::boxplus 的右乘扰动约定 rot = R̂·exp(δ^) 一致）：
 *     u_body  = (a_meas - ba) / |a_meas - ba|          // 静止/低动态时 = IMU 系里的"上"
 *     h(x)    = (0,0,1) - R·u_body                     // 代价（应为 0）
 *     J       = ∂h/∂δ = R·hat(u_body)                  // 只对姿态 3 维非零
 *     HTH     = JᵀJ = hat(u_body)ᵀhat(u_body)          // 天然秩 2（绕重力方向自转不可观测 ⇒ yaw 不被约束）
 *     HTr     = Jᵀ(-h) = hat(u_body)·Rᵀ(0,0,1)         // 与本工程 LiDAR 观测同符号约定（res = -h）
 *
 * 权重：w = gravity_obs_sigma_scale / σ_rad²，σ 默认 1.5°。
 *   量级对比：一帧 LiDAR 约 2000 个内点、σ≈2 cm ⇒ 信息量 ~5e6；本观测 w ≈ 1460。
 *   即它比 LiDAR 弱约 3 个数量级 ⇒ 只在 LiDAR 约束不到的（退化/长直/空旷）方向上起"慢修正"作用，
 *   不会与 LiDAR 抢主导权，也不改变正常路段的匹配精度。
 *
 * 低动态门限：| |a|/g - 1 | < gravity_obs_acc_tol_ 且 |ω| < gravity_obs_gyr_tol_
 *   （转弯/制动时比力方向 ≠ 竖直方向，此时不给观测，避免把车体姿态也一起带歪）
 */
void LaserMapping::AccAsGravityObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    obs.HTH_.setZero();
    obs.HTr_.setZero();
    obs.lidar_residual_mean_ = 0;
    obs.valid_ = false;  // 默认不施加，满足所有门限后再置 true

    if (!gravity_obs_enable_ || measures_.imu_.empty()) {
        return;
    }

    // 个人修改（方案C，2026-10-02 修正）：用【当前 scan 窗口的均值】而不是最后一个采样。
    //   实测：单采样的 |ω| 很容易超过低动态门限（表中 0.238 rad/s），导致重力观测几乎每帧被跳过、
    //   形同虚设（日志里 d_rot 只有 1e-4 度）；窗口均值（10~20 个采样）把噪声降到 ~1/4。
    Vec3d a_body;
    Vec3d w_body;
    double acc_dir_spread_deg = 0.0;
    if (!GetWindowImuMean(&a_body, &w_body, &acc_dir_spread_deg)) {
        return;
    }
    if (!a_body.allFinite() || a_body.norm() < 1e-3) {
        return;
    }

    // 个人修改（方案C，2026-10-02 加固）：窗口太短（采样太少）时"方向散布"没有统计意义（单采样恒为 0），
    //   此时不用它做判定，也不施加观测，避免把偶然的单点当成"静止"。
    if (measures_.imu_.size() < 5) {
        return;
    }

    const double acc_rel = std::fabs(a_body.norm() / G_m_s2 - 1.0);
    const double gyr_norm = w_body.norm();

    // =============================================================================================
    // 个人修改（方案C，2026-10-02 第4次修复①）：【行驶中的竖直锚】—— 本次 roll 漂移的根治点。
    //
    //   原实现只有一套门限（含"窗口比力方向散布 ≤ 3°"），实测在【车一动】时几乎恒被撞穿：
    //     日志 `gravity obs skipped (dynamic): |a|/g-1 = 0.0057, |w| = 0.0047 rad/s, 方向散布 = 3.20 deg`
    //     ⇒ 本函数返回 valid_ = false ⇒ ESKF 把这次更新整体回滚（日志表现为 d_rot = 0 deg，
    //       而同一时刻"世界系夹角(无ba) = 4.77°"）⇒ 行驶全程姿态没有绝对竖直参考。
    //   后果（本 bag 实测）：陀螺零偏（bg_z = 0.0416 rad/s = 2.39°/s，正是 roll 轴）在运动中可被
    //     重新随机游走（b_gyr_cov = 5e-4）⇒ roll 从 0 积到 +18.97°（同段陀螺证明车体只动 ≤0.79°）
    //     ⇒ 世界系倾斜 ⇒ z 沿路径下弓 1.37 m；一停车重力观测重新生效，|z| 又回到 0.026 m。
    //
    //   现在按"车是否在动"选两套门限：
    //     · 静止/准静止（|v| ≤ gravity_obs_motion_min_speed_mps_）：沿用严格门限（σ = 1.5°）；
    //     · 行驶中：必须【能证明此刻比力 ≈ 纯重力】才施加（σ = 3.0°，更软）：
    //         ① | |a|/g-1 | ≤ acc_tol、② |ω| ≤ gyr_tol、③ 平动加速度 ≤ motion_acc_tol（**必须有效**）、
    //         ④ 方向散布 ≤ gravity_obs_motion_acc_dir_tol_deg_（放宽到 10°；振动是随机的，
    //            窗口均值误差 ≪ 散布本身，而"系统性假重力"由③负责挡）。
    //       ③ 是关键：恒定的加/减速或转弯侧向加速度会造出 15° 级假重力，而 |a|、|ω|、方向散布
    //       三项全都看不出来（见 UpdateMotionAccEstimate 的说明）；LIO 速度差分与加计无关，能看出来。
    //   ★安全性：所有门限同时满足才施加，且 σ 更大（更软）+ 仍远弱于 LiDAR（信息量差 3 个数量级）
    //     ⇒ 最坏情况也只是"每帧 2~3% 的慢修正"，不会与 LiDAR 抢主导权、不会突变。
    // =============================================================================================
    const double speed = s.vel_.allFinite() ? s.vel_.norm() : 0.0;
    const bool moving = speed > gravity_obs_motion_min_speed_mps_;

    double sigma_deg = gravity_obs_sigma_deg_;
    double dir_tol_deg = gravity_obs_acc_dir_tol_deg_;
    if (moving) {
        if (!gravity_obs_motion_enable_) {
            gravity_obs_last_mode_ = 0;
            gravity_obs_skipped_++;
            return;
        }
        // ③ 平动加速度门限（LIO 速度差分）：历史不足一个基线（motion_acc_valid_ = false）时也【不施加】——
        //    那时没有任何独立手段证明"比力 = 纯重力"，宁可这一帧不给锚。
        if (!motion_acc_valid_ || motion_acc_est_ > gravity_obs_motion_acc_tol_) {
            gravity_obs_last_mode_ = 0;
            gravity_obs_skipped_++;
            LOG_EVERY_N(INFO, 200) << "[方案C] gravity obs skipped (moving, 无法证明比力=纯重力): LIO 平动加速度 = "
                                   << (motion_acc_valid_ ? motion_acc_est_ : -1.0) << " m/s² (有效="
                                   << (motion_acc_valid_ ? 1 : 0) << ", 门限 " << gravity_obs_motion_acc_tol_
                                   << " m/s²), |v| = " << speed << " m/s";
            return;
        }
        sigma_deg = gravity_obs_motion_sigma_deg_;
        dir_tol_deg = gravity_obs_motion_acc_dir_tol_deg_;
    }

    // 个人修改（方案C，2026-10-02 加固）：除幅值与角速度外，还必须要求【窗口内比力方向稳定】
    //   （否则横向加速度的"假重力"会把姿态带歪，实测可偏 15°）
    if (acc_rel > gravity_obs_acc_tol_ || gyr_norm > gravity_obs_gyr_tol_ || acc_dir_spread_deg > dir_tol_deg) {
        gravity_obs_last_mode_ = 0;
        gravity_obs_skipped_++;
        LOG_EVERY_N(INFO, 100) << "[方案C] gravity obs skipped (dynamic): |a|/g-1 = " << acc_rel
                               << " (tol " << gravity_obs_acc_tol_ << "), |w| = " << gyr_norm << " rad/s (tol "
                               << gravity_obs_gyr_tol_ << "), 窗口比力方向散布 = " << acc_dir_spread_deg
                               << " deg (tol " << dir_tol_deg << " deg), |v| = " << speed
                               << " m/s ⇒ 区分: " << (moving ? "行驶中" : "准静止");
        return;
    }

    const Vec3d up_body = (a_body - s.ba_).normalized();
    const Vec3d up_world = s.rot_ * up_body;
    const Vec3d r = up_world - Vec3d(0, 0, 1);  // r = -h
    const Mat3d J = s.rot_.matrix() * SO3::hat(up_body);

    const double sigma_rad = std::max(sigma_deg, 0.05) * M_PI / 180.0;
    const double w = gravity_obs_sigma_scale_ / (sigma_rad * sigma_rad);

    obs.HTH_.block<3, 3>(3, 3) = w * (J.transpose() * J);
    obs.HTr_.block<3, 1>(3, 0) = w * (J.transpose() * r);
    obs.lidar_residual_mean_ = r.norm();
    obs.valid_ = true;

    // 个人修改（方案C，2026-10-02 第4次修复①）：记下本次施加的是哪一套门限 + 计数，便于验收
    //   （验收要求：log 里能看到"行驶中"也在施加竖直锚，而不是全程只有静止段才施加）。
    gravity_obs_last_mode_ = moving ? 2 : 1;
    if (moving) {
        gravity_obs_applied_motion_++;
        // 当前"姿态所定义的世界系与重力"的夹角（与 CheckGravityAlignment 同一算法：夹紧后再 acos）
        const double cos_tilt_here = std::min(1.0, std::max(-1.0, up_world.normalized().dot(Vec3d(0, 0, 1))));
        LOG_EVERY_N(INFO, 100) << "[方案C] gravity obs(行驶中竖直锚) 已施加: |v| = " << speed
                               << " m/s, LIO 平动加速度 = " << motion_acc_est_ << " m/s², |a|/g-1 = " << acc_rel
                               << ", |w| = " << gyr_norm << " rad/s, 方向散布 = " << acc_dir_spread_deg << " deg"
                               << ", sigma = " << sigma_deg << " deg, 当前世界系夹角 = "
                               << std::acos(cos_tilt_here) * 180.0 / M_PI << " deg (累计 " << gravity_obs_applied_motion_
                               << " 次)";
    } else {
        gravity_obs_applied_static_++;
    }
}

/**
 * 个人修改（方案C）：两个【必须已归一化】矢量的夹角（度）。
 */
static double AngleDegBetween(const Vec3d &u, const Vec3d &v) {
    const double c = std::min(1.0, std::max(-1.0, u.dot(v)));
    return std::acos(c) * 180.0 / M_PI;
}

/**
 * 个人修改（方案C）：计算"姿态所定义的世界系 z 轴"与实测重力方向的夹角（度）。
 *   它等于"车水平直行时轨迹在世界系里的倾角"，也是 z 漂移的直接原因。
 *   返回值 ≈0 表示世界系竖直（正常）；>2~3° 说明世界系已经倾斜，必须处理。
 *
 * 个人修改（方案C，2026-10-02 加固①）：只允许【准静态窗口】给出这个数字。
 *   原实现取"最后一个采样的比力"，于是转弯/过坎时读到的是【加计的横向加速度】而不是姿态误差
 *   （本 bag 实测 15.33° @|a|=8.68 m/s²，横向仅 1.13 m/s²），会让人误判"世界系歪了 15°"。
 *   加固后：动态窗口直接跳过（保留上一次准静态值），日志明确写"动态窗口，跳过自检"。
 *
 * 个人修改（方案C，2026-10-02 加固②）：判据改用【未扣 ba 的原始比力方向 a】而不是 (a - ba)。
 *   理由（本 bag 实测）：ESKF 的加计零偏 ba 会被机动"拽"到 |ba| ≈ 1.2 m/s²（≈7°），
 *   而 ba 观测性极差（b_acc_cov 很小 ⇒ 学到后几乎不回弹）⇒ 一旦被污染就长期保持错值。
 *   此时 (a - ba) 方向会偏 7°，让本函数读出 4.9~12.9° 的"世界系歪了"假警报，而同一时刻：
 *     · 地图地面倾角（世界系内测得）= 21.05°；物理实测地面与重力夹角（LiDAR+原始加计，与 ba 无关）= 22.77°
 *     ⇒ 世界系其实只歪 ~1.7°，那个大数字纯粹来自 ba。
 *   准静态窗口里 |a| ≈ g 且方向稳定 ⇒ 原始比力方向本身就等于重力方向（与 ba 无关，干净）。
 *   带 ba 的值仍会打印出来做诊断：两者差得多 ⇒ ba 被污染（阈值 gravity_obs_ba_warn_deg）。
 */
double LaserMapping::CheckGravityAlignment(const NavState &s, bool print) {
    Vec3d a_body;
    Vec3d w_body;
    double acc_dir_spread_deg = 0.0;
    const bool has_window = GetWindowImuMean(&a_body, &w_body, &acc_dir_spread_deg);
    // 采样太少时"方向散布"没有统计意义（单采样恒为 0），此时不给自检数字
    const bool long_enough = has_window && measures_.imu_.size() >= 5;
    bool quasi_static = long_enough && a_body.allFinite() && a_body.norm() > 1e-3;
    if (quasi_static) {
        const double acc_rel = std::fabs(a_body.norm() / G_m_s2 - 1.0);
        quasi_static = (acc_rel <= gravity_obs_acc_tol_ && w_body.norm() <= gravity_obs_gyr_tol_ &&
                        acc_dir_spread_deg <= gravity_obs_acc_dir_tol_deg_);
    }

    // 个人修改（方案C，2026-10-02 第3次加固）：自检还要求【平动加速度小】—— 用 LIO 速度差分（与加计无关）。
    //   加计自己的三个门限都看不出"恒定的加/减速"：本 bag 实测在真正静止时读到 0.26°，
    //   而同一次运行里车辆加速的那一瞬间同一个自检读到 15.58°（差 60 倍）⇒ 必须先剔掉这种样本。
    const bool motion_ok = !motion_acc_valid_ || motion_acc_est_ <= gravity_obs_motion_acc_tol_;
    if (quasi_static && !motion_ok) {
        quasi_static = false;
    }

    if (!quasi_static) {
        if (print) {
            LOG(INFO) << "[方案C][self-check] 当前不是准静态窗口（|a| = "
                      << (has_window ? a_body.norm() : 0.0) << ", |w| = " << (has_window ? w_body.norm() : 0.0)
                      << " rad/s, 比力方向散布 = " << acc_dir_spread_deg << " deg, 平动加速度 = "
                      << (motion_acc_valid_ ? std::to_string(motion_acc_est_) : std::string("未知/历史不足"))
                      << " m/s²(LIO 速度差分)）：跳过本次自检；判定仍用【累积均值】"
                      << ((gravity_dir_world_count_ > 0)
                              ? AngleDegBetween(gravity_dir_world_sum_.normalized(), Vec3d(0, 0, 1))
                              : 0.0)
                      << " deg（" << gravity_dir_world_count_ << " 个已确认静止样本；最近一个样本 "
                      << gravity_misalign_deg_ << " deg）";
        }
        return gravity_misalign_deg_;
    }

    // 无 ba 的竖直方向：准静态窗口里 |a| ≈ g 且方向稳定 ⇒ 原始比力方向 = 重力方向（干净、可作判据）
    const Vec3d up_body = a_body.normalized();
    const Vec3d up_world = (s.rot_ * up_body).normalized();
    const double tilt_deg = AngleDegBetween(up_world, Vec3d(0, 0, 1));
    // 个人修改（方案C，2026-10-02 第3次加固）：这里【不再】把单帧值写进 gravity_misalign_deg_。
    //   原因：gravity_misalign_deg_ 现在只表示【累积均值】（"世界系再对齐"与各处日志的判定依据），
    //   而单帧值即使过了所有门限也可能被瞬时冲击污染（本 bag 实测有 15.58° 的单帧值与真值 0.26°
    //   差 60 倍）。混用会让日志自相矛盾、误导排查。

    // 带 ba 的同项（仅诊断）：= 真实世界系倾斜 ⊕ ba 等效倾角，绝不能单独当"世界系歪了"用
    const Vec3d up_body_ba = (a_body - s.ba_).normalized();
    const double tilt_ba_deg = AngleDegBetween((s.rot_ * up_body_ba).normalized(), Vec3d(0, 0, 1));
    const double ba_equiv_deg = AngleDegBetween(up_body, up_body_ba);

    if (print) {
        const SE3 T_w_vehicle = vehicle_frame::ToVehiclePose(s.GetPose());
        const PoseRPYD rpy = math::SE3ToRollPitchYaw(T_w_vehicle);
        LOG(INFO) << "[方案C][self-check] 世界系与重力夹角（单帧）= " << tilt_deg
                  << " deg (准静态窗口, 无 ba, up_world = " << up_world.transpose() << "), 车体 rpy(deg) = ["
                  << rpy.roll * 180.0 / M_PI << ", " << rpy.pitch * 180.0 / M_PI << ", " << rpy.yaw * 180.0 / M_PI
                  << "]"
                  << ", 平动加速度 = " << (motion_acc_valid_ ? motion_acc_est_ : 0.0)
                  << " m/s²(LIO 速度差分, 已确认小于门限 " << gravity_obs_motion_acc_tol_ << ")"
                  << ", |v| = " << s.vel_.norm() << " m/s"
                  << "（>2~3° 即说明世界系已倾斜、z 会沿路径漂移；本平台正常应 <1°。"
                  << "注意：本值只反映【姿态与世界重力】的关系，若车真在坡道上，地图地面倾斜是正常的）"
                  << "；★判定请以【累积均值】为准：均值 "
                  << ((gravity_dir_world_count_ > 0)
                          ? AngleDegBetween(gravity_dir_world_sum_.normalized(), Vec3d(0, 0, 1))
                          : 0.0)
                  << " deg / " << gravity_dir_world_count_ << " 个已确认静止样本（最近一个样本 "
                  << gravity_misalign_deg_
                  << " deg；单帧值可能被平动加速度/瞬时冲击污染——本 bag 实测单帧 15.58° vs 真值 0.26°，差 60 倍）";
        if (ba_equiv_deg > gravity_obs_ba_warn_deg_) {
            LOG(WARNING) << "[方案C][self-check] 带 ba 的同项 = " << tilt_ba_deg << " deg，比无 ba 的大 "
                         << ba_equiv_deg << " deg（|ba| = " << s.ba_.norm()
                         << " m/s²）⇒ 这段差值来自【加计零偏 ba 被机动污染（观测性差、几乎不回弹）】，"
                         << "不是世界系倾斜；自检与世界系再对齐判据已用无 ba 的原始比力方向。"
                         << "排查 z 漂移时请勿采信带 ba 的数字；必要时调大 eskf 的 b_acc_cov 让 ba 能回归。";
        }
    }

    return tilt_deg;
}

/**
 * 个人修改（方案C）：当前 scan 窗口内的 IMU 均值（比力已乘量纲因子；角速度已扣陀螺零偏 bg）。
 *   静止/低动态判据与重力方向都用它 ⇒ 比"最后一个采样"稳得多（噪声 ~1/4）。
 *   个人修改（方案C，2026-10-02 加固）：另外输出窗口内【各采样比力方向相对均值方向的最大夹角(度)】。
 *     只用 |a| 幅值判定是不够的：横向加速度（转弯/过坎）会让 |a| 几乎不变而方向偏 10~15°，
 *     这种"假重力"一旦被当成竖直参考就会把姿态带歪 ⇒ 必须同时检查方向的稳定性。
 */
bool LaserMapping::GetWindowImuMean(Vec3d *acc_mean, Vec3d *gyr_mean, double *acc_dir_spread_deg) const {
    if (measures_.imu_.empty()) {
        return false;
    }

    Vec3d a = Vec3d::Zero();
    Vec3d w = Vec3d::Zero();
    for (const auto &imu : measures_.imu_) {
        a += imu->linear_acceleration;
        w += imu->angular_velocity;
    }

    const double n = static_cast<double>(measures_.imu_.size());
    // 个人修改（2026-10-09 第6次修复，方案D）：先按 imu_acc_up_sign 定向（静止时朝上），与 IMUInit() 的
    //   世界系定义一致；否则本平台的重力观测/自检/世界系再对齐会把 -g 当成竖直参考（差 180°）。
    const double up_sign = p_imu_->GetAccUpSign();
    a *= up_sign * p_imu_->GetAccScaleFactor() / n;
    w /= n;

    // 陀螺零偏只为门限判定服务（零偏不该占掉"低动态"的额度）
    const Vec3d bg = kf_.GetX().bg_;
    w -= bg;

    // 个人修改（方案C，2026-10-02 加固）：方向散布 = max_i angle( â_i , â_mean )
    if (acc_dir_spread_deg != nullptr) {
        double spread = 0.0;
        if (a.norm() > 1e-6) {
            const Vec3d a_dir = a.normalized();
            for (const auto &imu : measures_.imu_) {
                const Vec3d ai = up_sign * imu->linear_acceleration;  // 个人修改（方案D）：与 a 同符号，否则散布恒为 180°
                if (ai.norm() < 1e-6) {
                    continue;
                }
                const double c = std::min(1.0, std::max(-1.0, ai.normalized().dot(a_dir)));
                spread = std::max(spread, std::acos(c));
            }
        }
        *acc_dir_spread_deg = spread * 180.0 / M_PI;
    }

    *acc_mean = a;
    *gyr_mean = w;
    return true;
}

/**
 * 个人修改（方案C）：累积"低动态样本的重力方向（世界系）"。
 *   用途：① 在线自检（日志能看到世界系何时开始歪、歪了多少）；
 *        ② 世界系重力再对齐（TryRealignWorldFrameToGravity）的唯一依据。
 *   只有低动态（| |a|/g-1 | 小、角速度小）样本才参与，避免把转弯/制动当成"重力方向"。
 */
/**
 * 个人修改（方案C，2026-10-02 第3次加固）：用【LIO 世界系速度差分】估计真实平动加速度（m/s²）。
 *
 * 为什么加计自身判据不够（本 bag 实测）：
 *   加计测的是比力 = 重力 + 平动加速度，所以【恒定的加/减速】会做出一个"假重力"方向，例如沿 22° 坡道以
 *   2.6 m/s² 加速时比力方向偏竖直 ~15°，而：① |a| 只变化 3.5%（acc_tol 0.15 放过）、
 *   ② 平稳加减速时 |ω| 很小（gyr_tol 0.15 rad/s 放过）、③ 加速度恒定时窗口内方向散布≈0（dir_tol 3° 放过）。
 *   实测：t=59.8 s 的"准静态"自检读到 15.58°，而同一时刻车辆正从静止加速到 0.3 m/s；对照 t=56.9 s
 *   （真静止：|v| = 0.003 m/s、|ω| = 0.001 rad/s、ba≈0）读数只有 0.26° ⇒ 大数字来自平动加速度。
 *   LIO 速度来自雷达/地图配准（与加计零偏无关）⇒ |Δv|/Δt 就是真实平动加速度，可用它把这类样本剔掉。
 *
 * 实现：把每帧的 (lidar_end_time, 世界系速度) 压入一个滚动历史，比较"现在"与"约 motion_window_s 之前"的
 *   速度差 ⇒ motion_acc_est_ = |Δv| / Δt。基线太短会被速度噪声淹没，太长会跨过机动，故默认 1.0 s。
 */
void LaserMapping::UpdateMotionAccEstimate(const NavState &s) {
    const double t = measures_.lidar_end_time_;
    if (!s.vel_.allFinite() || !std::isfinite(t)) {
        motion_acc_valid_ = false;
        return;
    }

    gravity_speed_hist_.emplace_back(t, s.vel_);
    const double baseline = std::max(0.2, gravity_obs_motion_window_s_);
    // 保证前端至少留 1 个"基线前"的样本：只在前面还有更老样本时才弹掉
    while (gravity_speed_hist_.size() > 2 && (t - gravity_speed_hist_.front().first) > baseline) {
        gravity_speed_hist_.pop_front();
    }
    // 历史维护：避免长时间运行后无限增长（保留 5 s 足够）
    while (!gravity_speed_hist_.empty() && (t - gravity_speed_hist_.front().first) > 5.0) {
        gravity_speed_hist_.pop_front();
    }

    const double dt = t - gravity_speed_hist_.front().first;
    if (gravity_speed_hist_.size() >= 2 && dt > 0.2) {
        motion_acc_est_ = (s.vel_ - gravity_speed_hist_.front().second).norm() / dt;
        motion_acc_valid_ = true;
    } else {
        motion_acc_valid_ = false;  // 历史不足一个基线：此时没有可信的平动加速度，门限判据不生效
    }
}

void LaserMapping::AccumulateGravityDirection(const NavState &s) {
    // 个人修改（方案C，2026-10-02 第3次加固）：先更新平动加速度估计（每帧一次），供下面各门限使用
    UpdateMotionAccEstimate(s);

    Vec3d a_body;
    Vec3d w_body;
    double acc_dir_spread_deg = 0.0;
    if (!GetWindowImuMean(&a_body, &w_body, &acc_dir_spread_deg)) {
        return;
    }
    if (!a_body.allFinite() || a_body.norm() < 1e-3 || measures_.imu_.size() < 5) {
        return;
    }

    const double acc_rel = std::fabs(a_body.norm() / G_m_s2 - 1.0);
    // 个人修改（方案C，2026-10-02 加固）：同样加上"窗口内比力方向稳定"判据 ——
    //   累积样本是【世界系再对齐】的唯一依据，一旦被横向加速度污染就会把好地图转歪。
    if (acc_rel > gravity_obs_acc_tol_ || w_body.norm() > gravity_obs_gyr_tol_ ||
        acc_dir_spread_deg > gravity_obs_acc_dir_tol_deg_) {
        return;
    }

    // 个人修改（方案C，2026-10-02 第3次加固）：平动加速度门限（来自 LIO 速度差分，与加计无关）。
    //   前三项判据都看不出"恒定的加/减速"（见 UpdateMotionAccEstimate 的说明），而它恰好能造出 15° 的假重力；
    //   本函数的结果会【刚性旋转整张地图】，所以这里必须最严格。
    if (motion_acc_valid_ && motion_acc_est_ > gravity_obs_motion_acc_tol_) {
        LOG_EVERY_N(INFO, 20) << "[方案C] 重力样本跳过（平动加速度 " << motion_acc_est_ << " m/s² > "
                              << gravity_obs_motion_acc_tol_ << " m/s²，来自 LIO 速度差分）：加计测的是比力，"
                              << "恒定加/减速会做出 15° 级“假重力”（|a|/|ω|/方向散布都看不出来），"
                              << "用它做世界系再对齐会把好地图整体转歪";
        return;
    }

    // 个人修改（方案C，2026-10-02 第3次加固）：世界系再对齐只采信【已确认静止】的样本。
    //   平动加速度门限挡的是"加/减速"，但还有一类同样能造出十几度假重力的样本：运动中撞到坎/坑、
    //   车身侧倾过坎等【瞬时冲击】—— 它的 1 s 平均 |Δv| 可能很小（先加速后减速，净变化≈0）、|ω| 小、
    //   窗口内方向散布也小，于是能骗过前面所有门限（本 bag 实测就有 15.6° 的样本通过了全部四项）。
    //   而本函数的结果会【刚性旋转整张地图】⇒ 判定依据必须是"车真的停着、比力就是重力"这种铁样本；
    //   静止样本本来在每个建图任务的起步段都有，够用。|v| 取 LIO 世界系速度（与加计无关）。
    //   设成很大（如 100）可关闭本判据（不建议）。
    if (gravity_obs_max_speed_mps_ > 0.0 && s.vel_.norm() > gravity_obs_max_speed_mps_) {
        LOG_EVERY_N(INFO, 20) << "[方案C] 重力样本跳过（LIO 速度 " << s.vel_.norm() << " m/s > "
                              << gravity_obs_max_speed_mps_ << " m/s，车在动）：世界系再对齐只用【静止】样本，"
                              << "因为运动中过坎/颠簸的瞬时冲击能做出 15° 级假重力且可能骗过其它门限";
        return;
    }

    // 个人修改（方案C，2026-10-02 加固②）：累积【未扣 ba 的原始比力方向】。
    //   本函数是世界系重力再对齐的【唯一依据】⇒ 绝不能被被污染的 ba 带偏（否则会把好地图整体旋转 7~13°；
    //   本 bag 实测差 1.6 ms 就真会按 12.9° 转地图）。准静态窗口里 |a| ≈ g ⇒ 原始比力方向 = 重力方向。
    const Vec3d up_body = a_body.normalized();
    const Vec3d up_world = (s.rot_ * up_body).normalized();
    if (!up_world.allFinite()) {
        return;
    }

    gravity_dir_world_sum_ += up_world;
    gravity_dir_world_count_++;

    gravity_misalign_deg_ = AngleDegBetween(up_world, Vec3d(0, 0, 1));

    // 诊断：带 ba 的版本偏了多少（偏得多 ⇒ ba 被污染，日志里能看出"假报警"的来源）
    const Vec3d up_body_ba = (a_body - s.ba_).normalized();
    const double ba_equiv_deg = AngleDegBetween(up_body, up_body_ba);
    if (ba_equiv_deg > gravity_obs_ba_warn_deg_) {
        LOG_EVERY_N(WARNING, 20) << "[方案C] 重力样本诊断: |ba| = " << s.ba_.norm() << " m/s² 使 (a-ba) 方向偏 "
                                 << ba_equiv_deg << " deg（带 ba 的夹角 "
                                 << AngleDegBetween((s.rot_ * up_body_ba).normalized(), Vec3d(0, 0, 1))
                                 << " deg）⇒ 该偏差来自 ba 污染，非世界系倾斜；累积与判据均用无 ba 方向。";
    }
}

/**
 * 个人修改（方案C）：用当前关键帧位姿重建 iVox 局部地图。
 *   世界系被刚性旋转后，局部地图里已有点云仍处于旧世界系 ⇒ 必须按新位姿重新灌入，
 *   否则后续帧的最近邻搜索会落在错位的墙/地面上，匹配会直接崩掉。
 */
void LaserMapping::RebuildLocalMapFromKeyframes() {
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    PointVector pts;
    for (auto &kf : all_keyframes_) {
        if (kf == nullptr || kf->GetCloud() == nullptr) {
            continue;
        }

        const NavState s = kf->GetState();
        const Mat3f R_wl = (s.rot_.matrix() * offset_R_lidar_fixed_).cast<float>();
        const Vec3f t_wl = (s.rot_ * offset_t_lidar_fixed_ + s.pos_).cast<float>();

        for (auto &pt : kf->GetCloud()->points) {
            PointType q;
            q.getVector3fMap() = R_wl * pt.getVector3fMap() + t_wl;
            q.intensity = pt.intensity;
            q.time = pt.time;
            pts.emplace_back(q);
        }
    }

    ivox_->AddPoints(pts);
    LOG(INFO) << "[方案C] 局部地图(iVox)已按新世界系重建: " << all_keyframes_.size() << " 个关键帧, " << pts.size()
              << " 个点";
}

/**
 * 个人修改（方案C）：世界系重力再对齐 —— 把整个建图结果刚性旋转回"z 轴竖直向上"。
 *
 *   触发条件（全部满足才动）：
 *     · 开关打开（fasterlio.world_frame_realign，默认 true）；
 *     · 已有 >= world_frame_realign_min_samples_ 个低动态重力样本（方向可信）；
 *     · 夹角在 [min_deg, max_deg] 内（默认 [0.7°, 30°]：太小不值得动，太大说明数据本身有问题，只告警）；
 *     · 仍处于建图早期（关键帧数 <= world_frame_realign_max_kf_ 且时长 <= max_time_s_）
 *       —— 此时地图尚未成形、重建代价小；超出窗口只告警，绝不悄悄改动已建好的地图。
 *
 *   做的事（一致地旋转同一个 R_fix，不引入任何形变）：
 *     ① ESKF 状态 kf_/kf_imu_：pos/rot 左乘 R_fix，协方差做对应相似变换；
 *     ② 所有关键帧：pose_lio_/pose_opt_/state_（点云由位姿间接表达，不用动）；
 *     ③ 局部地图 iVox：按新位姿重建；
 *     ④ 回调通知外部（slam.cc → g2p5::RedrawGlobalMap()）重绘 2.5D 栅格图；
 *     ⑤ state_point_（本帧对外状态）同步。
 *   注意：ba/bg（IMU 系零偏）与 grav_（世界系重力常量）与世界系旋转无关，保持不变。
 */
void LaserMapping::TryRealignWorldFrameToGravity() {
    if (!world_frame_realign_enable_ || world_frame_realigned_ || gravity_dir_world_count_ <= 0) {
        return;
    }

    // 个人修改（方案C）：只在【建图】模式下做世界系再对齐。
    //   理由：定位端（is_in_slam_mode_=false）不保存关键帧（all_keyframes_ 为空），
    //   它的 iVox 局部地图是由每帧 scan 增量灌进去的；整体旋转位姿会让"位姿与局部地图"错位，
    //   而定位端的 LIO 世界系只是【里程计基准】（对外只用相对量 delta），本来就不需要竖直。
    //   定位端的绝对参考是地图文件，由 CheckMapLevel() 负责自检。
    if (!options_.is_in_slam_mode_) {
        world_frame_realigned_ = true;
        LOG(INFO) << "[方案C] 当前为定位端 LIO（is_in_slam_mode=0）：世界系只是里程计基准，跳过重力再对齐"
                  << "（自检值：世界系与重力夹角 " << gravity_misalign_deg_ << " deg）";
        return;
    }

    // 建图模式下，本函数在 Run() 里位于 MakeKF() 之前 ⇒ 第一帧时关键帧列表还是空的。
    // 注意：这里【不能】置位 world_frame_realigned_，否则第一帧就把再对齐永久关掉了。
    if (all_keyframes_.empty()) {
        return;
    }

    // 个人修改（方案C，2026-10-02 第3次加固）：先用【累积的均值方向】算出判定用的夹角，
    //   再做窗口/样本数检查 —— 这样所有日志打印的都是"真正参与判定的那个数"。
    //   原实现把"最近一次自检值"（单个样本，可能被平动加速度污染成 15.6°）打印出来，
    //   而实际判定用的是累积均值，两者混在一起会让人误读（本 bag 实测就差 60 倍）。
    const Vec3d up_world = gravity_dir_world_sum_.normalized();
    if (!up_world.allFinite() || up_world.norm() < 1e-6) {
        return;
    }

    const double cos_tilt = std::min(1.0, std::max(-1.0, up_world.dot(Vec3d(0, 0, 1))));
    const double tilt_deg = std::acos(cos_tilt) * 180.0 / M_PI;

    // 超出允许的"早期窗口" ⇒ 放弃再对齐（只保留自检日志），避免对已成形地图做整体改动
    const double elapsed =
        (first_lidar_time_ > 0 && measures_.lidar_end_time_ > 0) ? (measures_.lidar_end_time_ - first_lidar_time_) : 0.0;
    if (kf_id_ > static_cast<unsigned long>(world_frame_realign_max_kf_id_) ||
        elapsed > world_frame_realign_max_time_s_) {
        world_frame_realigned_ = true;
        LOG(WARNING) << "[方案C] 世界系与重力夹角（累积均值，" << gravity_dir_world_count_ << " 个低动态样本）= "
                     << tilt_deg << " deg，但已超出世界系再对齐窗口（kf=" << kf_id_ << ", elapsed=" << elapsed
                     << "s；上限 kf<=" << world_frame_realign_max_kf_id_ << " 且 <=" << world_frame_realign_max_time_s_
                     << "s），本次不再自动旋转。请检查建图起点的静止条件，并考虑重建地图。"
                     << "（参考：最近一个被接受的静止样本 " << gravity_misalign_deg_
                     << " deg；单帧/单样本值可能被平动加速度污染，判定一律以累积均值为准）";
        return;
    }

    if (gravity_dir_world_count_ < world_frame_realign_min_samples_) {
        return;
    }

    if (tilt_deg < world_frame_realign_min_deg_) {
        world_frame_realigned_ = true;
        LOG(INFO) << "[方案C] 世界系与重力夹角 " << tilt_deg << " deg < " << world_frame_realign_min_deg_
                  << " deg：世界系竖直，无需再对齐（低动态样本 " << gravity_dir_world_count_ << " 个）";
        return;
    }

    if (tilt_deg > world_frame_realign_max_deg_) {
        world_frame_realigned_ = true;
        LOG(ERROR) << "[方案C] 世界系与重力夹角 " << tilt_deg << " deg 超过阈值 " << world_frame_realign_max_deg_
                   << " deg（低动态样本 " << gravity_dir_world_count_ << " 个）：这通常意味着"
                   << "①建图起点车辆在运动（IMU 初始化窗口被运动加速度污染）、或"
                   << "②IMU/雷达外参或时间戳错位等更基础的问题。本次【不自动旋转】以免掩盖故障，"
                   << "请先排查（看上面的 [imu init] / self-check 日志）；若确认只是初始对齐问题，请静止重跑建图。";
        return;
    }

    // 最小旋转：把"实测重力方向（世界系）"转到 +z —— 等价于同时旋转全体位姿/点云，
    // 地图形状与轨迹形状完全不变，只是把世界系基准扶正
    const Quatd q_fix = Quatd::FromTwoVectors(up_world, Vec3d(0, 0, 1));
    const Mat3d R_fix = q_fix.toRotationMatrix();

    // ① ESKF 状态（pos/vel 是"世界系矢量"⇒ 左乘 R_fix；rot/bg/ba 是局部量 ⇒ 不变）
    auto rotate_eskf = [&](ESKF &kf) {
        NavState s = kf.GetX();
        s.SetPose(SE3(R_fix * s.rot_.matrix(), R_fix * s.pos_));
        kf.ChangeX(s);

        ESKF::CovType P = kf.GetP();
        ESKF::CovType T = ESKF::CovType::Identity();
        T.block<3, 3>(NavState::kPosIdx, NavState::kPosIdx) = R_fix;
        T.block<3, 3>(NavState::kVelIdx, NavState::kVelIdx) = R_fix;
        P = T * P * T.transpose();
        kf.ChangeP(P);
    };
    rotate_eskf(kf_);
    rotate_eskf(kf_imu_);

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;

    // ② 所有关键帧（分别保持 lio / opt 两个位姿各自的含义）
    for (auto &kf : all_keyframes_) {
        if (kf == nullptr) {
            continue;
        }
        const SE3 old_lio = kf->GetLIOPose();
        const SE3 old_opt = kf->GetOptPose();
        kf->SetLIOPose(SE3(R_fix * old_lio.so3().matrix(), R_fix * old_lio.translation()));
        kf->SetOptPose(SE3(R_fix * old_opt.so3().matrix(), R_fix * old_opt.translation()));

        NavState s = kf->GetState();
        s.SetPose(SE3(R_fix * s.rot_.matrix(), R_fix * s.pos_));
        kf->SetState(s);
    }

    // ③ 局部地图按新世界系重建
    RebuildLocalMapFromKeyframes();

    // ④ 通知外部（g2p5 2.5D 栅格图）重绘
    if (world_frame_fixed_cb_) {
        world_frame_fixed_cb_();
    }

    world_frame_realigned_ = true;
    gravity_dir_world_sum_.setZero();
    gravity_dir_world_count_ = 0;
    gravity_misalign_deg_ = 0.0;

    LOG(WARNING) << "[方案C] 已执行【世界系重力再对齐】：把位姿/关键帧/局部地图整体旋转 " << tilt_deg
                 << " deg，使世界系 z 轴与实测重力一致（原 up_world = " << up_world.transpose()
                 << "）。这消除了既有轨迹/地图的整体倾斜（z 沿路径漂移的直接原因），几何形状未变。";
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}


void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    publish_count_++;

    double timestamp = imu->timestamp;

    UL lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    if (p_imu_->IsIMUInited()) {
        // 个人修改（2026-10-09，DR 时间戳回退 ⇒ 定位卡住/移动不丝滑）：
        //   ESKF::Predict() 内部只做 x_.timestamp_ += dt（增量时间），它隐含要求"kf_imu_ 当前时间
        //   就是上一次传给它的时标基准"。但 Run() 末尾的 kf_imu_ = kf_ 会把时间基重置到雷达帧末
        //   (pcl_end_time)，而这里传的 dt = 本帧IMU时标 - 上一帧IMU时标（IMU 时间轴上的增量）
        //   ⇒ 时间基错位，DR 输出时间戳整体偏离真实 IMU 时标，甚至小于上一帧；
        //   PGO::ProcessDR 判定"时间戳回退"后会丢弃该帧，连续丢弃就让高频定位输出整段停更
        //   （实机表现：刷"当前DR定位的结果的时间戳应当比上一个时间戳数值大"的同时定位卡住）。
        //   这里① 只在正向时间间隔内预测（避免用负 dt 把状态"倒推"）；
        //        ② 每次预测后显式把状态时间对齐到当前 IMU 时标，
        //           使 DR 时间戳恒等于 IMU 采样时标（与雷达/LO 同一时钟，可直接参与插值/外推）。
        const double dt = timestamp - last_timestamp_imu_;
        if (dt > 0) {
            kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity,
                            // 个人修改（2026-10-09 第6次修复，方案D）：kf_imu_ 的世界系与 kf_ 相同（z=物理上），
                            //   因此这里喂给预测的比力必须与 IMUInit/UndistortPcl 用同一个符号，否则 v̇ 变成 -2g。
                            p_imu_->GetAccUpSign() * imu->linear_acceleration);
        } else {
            LOG_EVERY_N(WARNING, 50) << "IMU 时间戳未前进(dt=" << dt << ")，本帧不做预测，仅同步时间基(累计 "
                                     << google::COUNTER << " 次)";
        }

        kf_imu_.SetTime(timestamp);

        // LOG(INFO) << "newest wrt lidar: " << timestamp - kf_.GetX().timestamp_;

        // 更新ui
        if (ui_) {
            // 个人修改（方案A+）：kf_imu_ 与 kf_ 同为"世界系下的 IMU 系位姿"，送 UI 前必须同样换算成
            //   车辆(base_link)系。否则每来一帧 IMU 状态就把 UI 里的车摆回 IMU 姿态
            //   （Airy 竖装表现为车绕 y 轴立起来 90°、pitch≈90°）。
            NavState state_vehicle = kf_imu_.GetX();
            state_vehicle.SetPose(vehicle_frame::ToVehiclePose(state_vehicle.GetPose()));
            ui_->UpdateNavState(state_vehicle);
        }
    }

    last_timestamp_imu_ = timestamp;

    imu_buffer_.emplace_back(imu);
}

bool LaserMapping::Run() {
    if (!SyncPackages()) {
        LOG(WARNING) << "sync package failed";
        return false;
    }

    // IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);

    // 个人修改（方案A+）：auto 模式下，IMU 首次初始化完成的那一刻就用其重力方向定出车体安装旋转，
    //   保证后面所有车辆系输出（UI/日志/TF）从第一帧起就是对的（函数内部自带幂等判断）。
    TryInitVehicleFrameFromGravity();

    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    // the first scan
    if (flg_first_scan_) {
        LOG(INFO) << "first scan pts: " << scan_undistort_->size();

        state_point_ = kf_.GetX();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        flg_first_scan_ = false;
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        if (skip_lidar_cnt_ != 0) {
            // 更新UI中的内容
            if (ui_) {
                // 个人修改：UI 中的姿态统一换算到【车辆(base_link)系】，位置不变、只换旋转的参考系。
                //   T_w_vehicle = T_w_body * Mᵀ（M = R_vehicle_imu，见 common/vehicle_frame.h）
                //   否则 IMU 竖装(x=下,y=左,z=前)会让 UI 里画的"车"像绕 y 轴立起来 90°。
                NavState state_vehicle = kf_.GetX();
                state_vehicle.SetPose(vehicle_frame::ToVehiclePose(state_vehicle.GetPose()));
                ui_->UpdateNavState(state_vehicle);

                // 个人修改：scan_undistort_ 是【雷达系】点云，必须用 T_w_lidar = T_w_body * T_imu_lidar 绘制。
                //   （原来直接把 IMU 姿态给了雷达系点云，整体差一个"雷达→IMU"外参，Airy 竖装雷达约 90°）
                ui_->UpdateScan(scan_undistort_, vehicle_frame::ToLidarPose(kf_.GetX().GetPose(), T_imu_lidar_extrinsic()));

                // 个人修改：车的姿态单独用车辆系姿态给出（UI 内部 UpdateScan 已不再改写 newest_backend_pose_）
                ui_->UpdateBodyPose(vehicle_frame::ToVehiclePose(kf_.GetX().GetPose()));
            }

            return false;
        }
    }

    LOG(INFO) << "=============================";
    LOG(INFO) << "LIO get cloud at beg: " << std::setprecision(14) << measures_.lidar_begin_time_
              << ", end: " << measures_.lidar_end_time_;

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LOG(ERROR) << "检测到雷达断流，时长：" << (measures_.lidar_begin_time_ - last_lidar_time_);
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    // downsample
    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    // if (options_.proj_kfs_) {
    //     ProjectKFs();
    // }

    int cur_pts = scan_down_body_->size();

    if (cur_pts < (scan_undistort_->size() * 0.1) || cur_pts < options_.min_pts) {
        // 降采样太狠了,有效点数不够，用0.1分辨率代替
        // LOG(INFO) << "too few points, using 0.1 resol";
        auto v = voxel_scan_;
        v.setLeafSize(0.1, 0.1, 0.1);
        v.setInputCloud(scan_undistort_);
        v.filter(*scan_down_body_);

        // LOG(INFO) << "Now pts: " << scan_down_body_->size() << ", before: " << cur_pts;
        cur_pts = scan_down_body_->size();
    }

    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }

    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);

    // 成员变量预分配
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, 1);
    point_selected_icp_.resize(cur_pts, 1);
    plane_coef_.resize(cur_pts, Vec4f::Zero());

    auto pred_state = kf_.GetX();
    // pred_state.pos_ = state_point_.pos_;  // 假定位置不动行不行,防止速度漂移
    // kf_.ChangeX(pred_state);

    kf_.Update(ESKF::ObsType::LIDAR, 1.0);

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;

    const double delta_translation = (pred_state.pos_ - state_point_.pos_).norm();
    const double delta_rotation_deg = (pred_state.rot_.inverse() * state_point_.rot_).log().norm() * 180.0 / M_PI;
    const double delta_velocity = (pred_state.vel_ - state_point_.vel_).norm();

    const double current_speed = state_point_.vel_.norm();

    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_surf_ << ", "
              << effect_feat_icp_;
    LOG(INFO) << "delta trans: " << (pred_state.pos_ - state_point_.pos_).transpose()
              << ", ang: " << delta_rotation_deg;
    // LOG(INFO) << "P diag: " << kf_.GetP().diagonal().transpose();

    // Vec3d v_from_last = (state_point_.pos_ - last_state.pos_) / (state_point_.timestamp_ - last_state.timestamp_);
    // LOG(INFO) << "v from last: " << v_from_last.transpose();

    // if (delta_velocity > 1.0 || current_speed > 4.0) {
    //     LOG(ERROR) << "detected very large vel change, last: " << last_state.vel_.transpose()
    //                << ", pred: " << pred_state.vel_.transpose() << ", cur:" << state_point_.vel_.transpose();
    //     LOG(ERROR) << "please check";
    // }

    // =============================================================================================
    //  个人修改（2026-10-02，方案C：姿态/重力根治）—— 姿态的绝对竖直参考 + 世界系再对齐
    //
    //  位置要求：必须在 MakeKF()/MapIncremental() 之前 ⇒ 关键帧位姿、局部地图、UI、以及
    //  送给定位端的投影点云全部使用"已被重力纠正过"的姿态，地图才不会带倾斜。
    // =============================================================================================
    AccumulateGravityDirection(state_point_);

    if (gravity_obs_enable_ && !measures_.imu_.empty() &&
        (gravity_obs_every_n_scan_ <= 1 || (scan_count_ % std::max(1, gravity_obs_every_n_scan_)) == 0)) {
        // 个人修改（方案C）：先给姿态协方差充气，否则这条"软观测"完全没作用 ——
        //   LiDAR 观测每帧把姿态协方差压到 ~0.03°（信息量 ~5e6），1.5° 的软观测增益只有 ~1e-4，
        //   单帧修正量 1e-4 度（实测就是这个数量级），等于没做。
        //   充气到 gravity_obs_prior_inflation_deg_（默认 0.5°）后增益约 0.1 ⇒ 每帧修正当前偏差的 10%，
        //   时间常数 ~1 s，既真正生效又不会突变（充的气也会被紧随其后的 LiDAR 观测重新收紧）。
        {
            ESKF::CovType P = kf_.GetP();
            const double sigma_inflate = gravity_obs_prior_inflation_deg_ * M_PI / 180.0;
            P.block<3, 3>(NavState::kRotIdx, NavState::kRotIdx) += sigma_inflate * sigma_inflate * Mat3d::Identity();
            kf_.ChangeP(P);
        }

        const NavState before = kf_.GetX();
        kf_.Update(ESKF::ObsType::ACC_AS_GRAVITY, 1.0);

        const NavState after = kf_.GetX();
        const double d_rot_deg = (before.rot_.inverse() * after.rot_).log().norm() * 180.0 / M_PI;
        state_point_ = after;
        state_point_.timestamp_ = measures_.lidar_end_time_;

        Vec3d acc_mean;
        Vec3d gyr_mean;
        GetWindowImuMean(&acc_mean, &gyr_mean);
        // 个人修改（方案C，2026-10-02 加固②）：这一行同时给出【无 ba】与【带 ba】两个"世界系与重力夹角"。
        //   无 ba 的那个才是可信的世界系倾斜（准静态窗口里原始比力方向 = 重力方向）；
        //   两者差值 = ba 的等效倾角，|ba| 被机动污染到 ~1.2 m/s²（≈7°）时就会造出 4.9~12.9° 的假警报。
        double tilt_noba_deg = 0.0;
        double tilt_ba_deg = 0.0;
        double ba_equiv_deg = 0.0;
        if (acc_mean.allFinite() && acc_mean.norm() > 1e-3) {
            const Vec3d up_noba = acc_mean.normalized();
            const Vec3d up_ba = (acc_mean - after.ba_).normalized();
            tilt_noba_deg = AngleDegBetween((after.rot_ * up_noba).normalized(), Vec3d(0, 0, 1));
            tilt_ba_deg = AngleDegBetween((after.rot_ * up_ba).normalized(), Vec3d(0, 0, 1));
            ba_equiv_deg = AngleDegBetween(up_noba, up_ba);
        }
        LOG_EVERY_N(INFO, 20) << "[方案C] gravity obs: d_rot = " << d_rot_deg
                              << " deg, 世界系夹角(无ba) = " << tilt_noba_deg << " deg, 带ba = " << tilt_ba_deg
                              << " deg (ba 等效倾角 " << ba_equiv_deg << " deg), ba = " << after.ba_.transpose()
                              << ", |a| = " << acc_mean.norm() << ", |w| = " << gyr_mean.norm() << ", 平动加速度 = "
                              << (motion_acc_valid_ ? motion_acc_est_ : 0.0) << " m/s²(LIO 差分)"
                              // 个人修改（方案C，2026-10-02 第4次修复①②）：模式 + bg。
                              //   · 模式 = "未施加(门限未过)" 且 d_rot = 0 而世界系夹角很大 ⇒ 姿态没有竖直锚
                              //     （这正是本次修复前的现象：行驶中方向散布 3.2° > 3° ⇒ valid_=false ⇒ 更新被回滚）；
                              //   · bg 是否恒定 = 修复②(禁止运动中随机游走零偏)是否生效的直接证据。
                              << ", 模式 = "
                              << (gravity_obs_last_mode_ == 2 ? "行驶中竖直锚"
                                                              : (gravity_obs_last_mode_ == 1 ? "准静止" : "未施加(门限未过)"))
                              << ", bg = " << after.bg_.transpose() << " (累计 行驶中=" << gravity_obs_applied_motion_
                              << " 准静止=" << gravity_obs_applied_static_ << " 跳过=" << gravity_obs_skipped_ << ")";
    }

    // 世界系自检（默认每 50 帧打一次）：能看到世界系何时开始歪、歪了多少
    if ((scan_count_ % 50) == 0) {
        CheckGravityAlignment(state_point_, true);
    }

    TryRealignWorldFrameToGravity();

    // keyframes
    if (last_kf_ == nullptr) {
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = state_point_.GetPose();
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } else if (!options_.is_in_slam_mode_ && (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        } else if ((last_pose.so3().inverse() * cur_pose.so3()).log().norm() > 1.0 * M_PI / 180.0) {
            // MapIncremental();
        }
    }

    // 更新kf_for_imu
    kf_imu_ = kf_;
    if (!measures_.imu_.empty()) {
        // 个人修改（2026-10-09，DR 时间戳回退 ⇒ 定位卡住/移动不丝滑）：
        //   kf_imu_ = kf_ 之后状态时间 = 雷达帧末(pcl_end_time)，而 imu_buffer_ 里装的是
        //   "sync 之后新到"的 IMU（时标都 > pcl_end_time）。原来基准 t 取 measures_.imu_.back()
        //   （<= pcl_end_time）会让第一个 dt 偏大（把这段运动重复积分一次），且积分结束时状态时间
        //   并不等于最新 IMU 时标 ⇒ DR 时间戳与 IMU 时间轴错位。
        //   这里改为从"状态自身时间"出发逐个 IMU 递推，并在结束时把状态时间对齐到最新一帧 IMU 时标，
        //   保证 kf_imu_ 的时间轴与 IMU 采样时标严格一致（DR 输出的时间戳因此单调且与雷达同一时钟）。
        double t = kf_imu_.GetX().timestamp_;
        for (auto &imu : imu_buffer_) {
            double dt = imu->timestamp - t;
            if (dt <= 0) {
                continue;
            }

            // 个人修改（2026-10-09 第6次修复，方案D）：同 ProcessIMU，kf_imu_ 的世界系 z 朝上 ⇒ 比力要同符号
            kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, p_imu_->GetAccUpSign() * imu->linear_acceleration);
            t = imu->timestamp;
        }

        kf_imu_.SetTime(t);
    }

    if (ui_) {
        // 个人修改：scan_down_body_ 同样是【雷达系】点云，用 T_w_lidar = T_w_body * T_imu_lidar 绘制；
        //   车体姿态（画车用）单独用【车辆(base_link)系】的 T_w_vehicle 给出。
        ui_->UpdateScan(scan_down_body_, vehicle_frame::ToLidarPose(state_point_.GetPose(), T_imu_lidar_extrinsic()));
        ui_->UpdateBodyPose(vehicle_frame::ToVehiclePose(state_point_.GetPose()));
    }

    // 个人修改：日志里的姿态也换算到【车辆(base_link)系】再取 RPY。
    //   原实现 state_point_.rot_.angleZ() 中的 SO3::angleZ() 是 2x2 子块的 SVD log，
    //   并不是真正的 yaw：IMU 竖装时得到的是"绕 y 轴"的退化结果（实测静止时旧值可达 -162°，
    //   而真实朝向只有几度），转弯时更是乱跳。
    //   现在用车辆系下的标准 ZYX 欧拉角（math::SE3ToRollPitchYaw，内部 tf2 getRPY）：
    //   roll 绕 x(前)、pitch 绕 y(左)、yaw 绕 z(上)，静止/直行时 roll≈pitch≈0，只有 yaw 随车头变化。
    const SE3 T_w_vehicle_log = vehicle_frame::ToVehiclePose(state_point_.GetPose());
    const PoseRPYD rpy_vehicle = math::SE3ToRollPitchYaw(T_w_vehicle_log);
    LOG(INFO) << "LIO state(vehicle/base_link): " << T_w_vehicle_log.translation().transpose() << ", rpy(deg): ["
              << rpy_vehicle.roll * 180.0 / M_PI << ", " << rpy_vehicle.pitch * 180.0 / M_PI << ", "
              << rpy_vehicle.yaw * 180.0 / M_PI << "]"
              << ", vel: " << state_point_.vel_.transpose() << ", grav: " << state_point_.grav_.transpose()
              << ", grav norm: " << state_point_.grav_.norm()
              // 个人修改（方案C，2026-10-02 第4次修复②）：把陀螺零偏搬到日志里。
              //   原日志没有 bg ⇒ 无法判断"零偏会不会在运动中重新随机游走"（本 bag 的 roll 漂移速率
              //   2.39°/s 与 bg_z 完全同量级，而日志里看不到它）⇒ 现在每帧可见，验收要求"全程基本恒定"。
              << ", bg: " << state_point_.bg_.transpose();

    return true;
}

void LaserMapping::ProjectKFs(CloudPtr cloud, int size_limit) {
    auto state = kf_.GetX();
    SE3 pose_cur(state.rot_, state.pos_);
    pose_cur = pose_cur.inverse();

    // 个人修改：投影进来的历史关键帧点云 kf->GetCloud() 是【雷达系】，而 pose_cur*kf->GetLIOPose()
    //   得到的是"当前 IMU 系 ← 历史 IMU 系"的相对位姿。要把【雷达系】点直接放到【当前雷达系】下，
    //   必须左右各补一次外参：
    //       T_lidar_cur_lidar_kf = T_imu_lidar⁻¹ * (T_imu_cur_w * T_w_imu_kf) * T_imu_lidar
    //   漏掉它的后果：投影进当前帧的历史关键帧点云整体歪一个 90° 外参（Airy 竖装平台），
    //   而这份点云正是定位端 LidarLoc 的输入（localization.cpp: lio_->GetProjCloud()），
    //   等于给定位喂了一份几何错乱的"局部地图"，会直接拉低匹配质量（甚至初始化失败）。
    const SE3 T_imu_lidar = T_imu_lidar_extrinsic();

    for (auto kf : proj_kfs_) {
        // LOG(INFO) << "projecting kf: " << kf->GetID();
        // if (last_kf_) {
        // auto kf = last_kf_;
        SE3 pose = T_imu_lidar.inverse() * pose_cur * kf->GetLIOPose() * T_imu_lidar;

        int cnt = 0;
        for (auto &pt : kf->GetCloud()->points) {
            Vec3d p = pose * ToVec3d(pt);
            PointType pcl_pt;

            pcl_pt.x = p.x();
            pcl_pt.y = p.y();
            pcl_pt.z = p.z();
            pcl_pt.intensity = pt.intensity;

            cloud->push_back(pcl_pt);
            cnt++;

            if (cnt > size_limit) {
                break;
            }
        }
        // }
    }
}

void LaserMapping::MakeKF() {
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_);

    if (last_kf_) {
        // opt pose 用之前的递推
        SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();
        kf->SetOptPose(last_kf_->GetOptPose() * delta);
    } else {
        kf->SetOptPose(kf->GetLIOPose());
    }

    kf->SetState(state_point_);

    LOG(INFO) << "LIO: create kf " << kf->GetID() << ", state: " << state_point_.pos_.transpose()
              << ", kf opt pose: " << kf->GetOptPose().translation().transpose()
              << ", lio pose: " << kf->GetLIOPose().translation().transpose() << ", time: " << std::setprecision(14)
              << state_point_.timestamp_;

    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;

    // 有keyframes时更新local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    // 更新project kfs
    if (proj_kfs_.size() >= options_.max_proj_kfs_) {
        auto last = proj_kfs_.back();

        SE3 delta = last->GetLIOPose().inverse() * kf->GetLIOPose();

        if (delta.translation().norm() < 3 || delta.so3().log().norm() < 20 / 180 * M_PI) {
            // proj_kfs_.pop_back();
        } else {
            proj_kfs_.pop_front();
            proj_kfs_.emplace_back(kf);
        }
    } else {
        proj_kfs_.emplace_back(kf);
    }

    // for (auto &kf : proj_kfs_) {
    //     LOG(INFO) << "proj kf: " << kf->GetID();
    // }
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, dt: " << timestamp - last_timestamp_lidar_;
                return;
            }

            LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
                      << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            // LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
            //           << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            double timestamp = math::ToSec(cloud->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        LOG(INFO) << "lidar or imu is empty";
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);

            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;

            if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
                // timestamp 有异常
                lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
                lidar_mean_scantime_ = lo::lidar_time_interval;
            }
        }

        lo::lidar_time_interval = lidar_mean_scantime_;

        // LOG(INFO) << "recompute lidar end time: " << std::setprecision(14) << lidar_end_time_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        LOG(INFO) << "sync failed: " << std::setprecision(14) << last_timestamp_imu_ << ", " << lidar_end_time_;
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());

        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // LOG(INFO) << "sync: " << std::setprecision(14) << measures_.lidar_begin_time_ << ", " <<
    // measures_.lidar_end_time_;

    return true;
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);  // FIXME 这并发可能有点问题
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // LOG(INFO) << "obs from state: " << s.pos_.transpose() << ", " << s.rot_.unit_quaternion().coeffs().transpose();

    Timer::Evaluate(
        [&, this]() {
            Mat3f R_wl = (s.rot_.matrix() * offset_R_lidar_fixed_).cast<float>();
            Vec3f t_wl = (s.rot_ * offset_t_lidar_fixed_ + s.pos_).cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                Vec3f p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                points_near.clear();

                /** Find the closest surfaces in the map **/
                ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;

                point_selected_icp_[i] = point_selected_surf_[i];

                // 能找到3个点以上，则估计平面
                if (point_selected_surf_[i]) {
                    point_selected_surf_[i] =
                        math::esti_plane(plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
                }

                // 计算平面阈值
                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    if (p_body.norm() > 81 * pd2 * pd2) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    } else {
                        point_selected_surf_[i] = false;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_surf_ = 0;
    effect_feat_icp_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_surf_] = plane_coef_[i];
            corr_pts_[effect_feat_surf_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_surf_][3] = residuals_[i];

            effect_feat_surf_++;
        }

        if (point_selected_icp_[i]) {
            effect_feat_icp_++;
        }
    }

    corr_pts_.resize(effect_feat_surf_);
    corr_norm_.resize(effect_feat_surf_);

    if (effect_feat_surf_ < 20) {
        obs.valid_ = false;
        LOG(WARNING) << "No enough effective surface points: " << effect_feat_surf_ << ", icp: " << effect_feat_icp_
                     << ", required: " << 20;
        return;
    }

    index.resize(effect_feat_surf_);
    const Mat3f off_R = offset_R_lidar_fixed_.cast<float>();
    const Vec3f off_t = offset_t_lidar_fixed_.cast<float>();
    const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

    // 点面ICP部分
    obs.HTH_.setZero();
    obs.HTr_.setZero();

    std::vector<Mat6d> JTJ(effect_feat_surf_);
    std::vector<Vec6d> JTr(effect_feat_surf_);

    std::vector<double> res_sq(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        Vec3f point_this_be = corr_pts_[i].head<3>();
        Vec3f point_this = off_R * point_this_be + off_t;
        Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        Vec3f norm_vec = corr_norm_[i].head<3>();

        /*** calculate the Measurement Jacobian matrix H ***/
        Vec3f C(Rt * norm_vec);
        Vec3f A(point_crossmat * C);

        Eigen::Matrix<double, 1, ESKF::pose_obs_dim_> J;
        J.setZero();
        J << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];

        float res = -corr_pts_[i][3];

        // double w = huber_weight(res);
        double w = 1.0;

        JTJ[i] = (J.transpose() * J).eval() * w;
        JTr[i] = J.transpose() * res * w;

        res_sq[i] = res * res;
    });

    for (int i = 0; i < index.size(); ++i) {
        obs.HTH_ += JTJ[i] * options_.plane_icp_weight_;
        obs.HTr_ += JTr[i] * options_.plane_icp_weight_;
    }

    if (!res_sq.empty()) {
        std::sort(res_sq.begin(), res_sq.end());
        obs.lidar_residual_mean_ = res_sq[res_sq.size() / 2];
        obs.lidar_residual_max_ = res_sq[res_sq.size() - 1];
        // LOG(INFO) << "residual mean: " << obs.lidar_residual_mean_ << ", max: " << obs.lidar_residual_max_
        //           << ", 85%: " << res_sq[res_sq.size() * 0.85];
    }

    // 点到点ICP部分

    if (options_.enable_icp_part_) {
        JTJ.resize(cnt_pts);
        JTr.resize(cnt_pts);

        std::vector<size_t> index(cnt_pts);
        for (size_t i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
            if (point_selected_icp_[i] == false) {
                return;
            }

            // TODO: 外参
            Vec3d q = scan_down_body_->points[i].getVector3fMap().cast<double>();
            Vec3d qs = scan_down_world_->points[i].getVector3fMap().cast<double>();

            Eigen::Matrix<double, 3, ESKF::pose_obs_dim_> J;
            J.setZero();

            // translation 部分
            J.block<3, 3>(0, 0) = Mat3d::Identity();

            // rotation 部分
            J.block<3, 3>(0, 3) = -(s.rot_.matrix() * offset_R_lidar_fixed_) * SO3::hat(q);

            Vec3d e = qs - nearest_points_[i][0].getVector3fMap().cast<double>();

            if (e.norm() > 0.5) {
                point_selected_icp_[i] = false;
                return;
            }

            JTJ[i] = J.transpose() * J;
            JTr[i] = -J.transpose() * e;
        });

        for (int i = 0; i < cnt_pts; ++i) {
            if (point_selected_icp_[i] == false) {
                continue;
            }
            obs.HTH_ += JTJ[i] * options_.icp_weight_;
            obs.HTr_ += JTr[i] * options_.icp_weight_;
        }
    }
}

//////////////////  private method //////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    // 个人修改：关键帧点云是【雷达系】(MakeKF 存入的 scan_undistort_ 位于雷达系)，
    // 而关键帧位姿是【IMU 系】；拼图时必须补上 lidar→IMU 外参，
    // 才能与前端 UI(见 Run 中的 ToLidarPose 用法)、局部地图、定位端(T_map_lidar)约定一致。
    const SE3 T_imu_lidar = T_imu_lidar_extrinsic();

    for (auto &kf : all_keyframes_) {
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        if (use_lio_pose) {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans,
                                     vehicle_frame::ToLidarPose(kf->GetLIOPose(), T_imu_lidar).matrix());
        } else {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans,
                                     vehicle_frame::ToLidarPose(kf->GetOptPose(), T_imu_lidar).matrix());
        }

        *global_map += *cloud_trans;

        LOG(INFO) << "kf " << kf->GetID() << ", pose: " << kf->GetOptPose().translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    LOG(INFO) << "global map: " << global_map_filtered->size();

    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    // 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    LOG(INFO) << "lio map is saved to ./data/lio.pcd";
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

CloudPtr LaserMapping::GetProjCloud() {
    // 个人修改（2026-10-02 修复段错误）：必须先【深拷贝】一份再投影，不能再写成 `auto cloud = scan_undistort_;`
    //   原因：MakeKF 里 `std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_)` 存的就是这个
    //   scan_undistort_（shared_ptr 别名，见 imu_processing.hpp `pcl_out = meas.scan_`），
    //   所以 `auto cloud = scan_undistort_` 之后 cloud 与 kf->GetCloud() 是【同一个对象】；
    //   而 ProjectKFs 是“边遍历边追加”：
    //       for (auto &pt : kf->GetCloud()->points) cloud->push_back(pcl_pt);
    //   二者落在同一个 points(vector) 上 ⇒ 第一次 push_back 就扩容
    //   （_M_realloc_insert：新块分配 + 拷贝 + 旧块 release），范围 for 的 begin/end 迭代器随即悬空，
    //   下一次解引用 pt 即 SIGSEGV（实测 gdb 回溯：ProjectKFs@laser_mapping.cc:1349 ← GetProjCloud:1883）。
    //   本平台 96 线单帧 86400 点 × 32B ≈ 2.7MB，超过 glibc mmap 阈值(128KB)，旧缓冲区会被 munmap，
    //   所以崩溃是【必现】的；若点云小于 128KB 则只是读出脏点（不崩但结果错）——更难查。
    //   触发条件：proj_kfs_ 非空，即“第一个关键帧创建后”的第一次 GetProjCloud()，与实测现象完全一致
    //   （MakeKF 打出 "LIO: create kf 0" → Run() 打出 "LIO state" → 紧接着 SIGSEGV）。
    //   另外这也顺带消除了 LIO 线程与定位线程共用 scan_undistort_ 的数据竞争
    //   （下一帧 UndistortPcl 会直接改写 scan_undistort_，而定位线程可能还在读）。代价仅一次 2.7MB 拷贝。
    CloudPtr cloud(new PointCloudType());
    *cloud = *scan_undistort_;  // pcl 点云 copy-assign：points/header/width/height 一并复制
    ProjectKFs(cloud);
    return cloud;
}

}  // namespace lightning
