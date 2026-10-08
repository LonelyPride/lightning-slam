#include <algorithm>
#include <cmath>
#include <execution>
#include <map>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/registration/ndt.h>

#include "pclomp/ndt_omp_impl.hpp"
#include "pclomp/voxel_grid_covariance_omp_impl.hpp"

#include "core/localization/lidar_loc/lidar_loc.h"

#include <opencv2/highgui.hpp>

#include "glog/logging.h"
#include "io/file_io.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"

namespace lightning::loc {

LidarLoc::LidarLoc(LidarLoc::Options options) : options_(options) {
    pcl_ndt_.reset(new NDTType());
    pcl_ndt_->setResolution(1.0);
    pcl_ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    pcl_ndt_->setOulierRatio(0.45);
    pcl_ndt_->setStepSize(0.1);
    pcl_ndt_->setTransformationEpsilon(0.01);
    pcl_ndt_->setMaximumIterations(20);
    pcl_ndt_->setNumThreads(4);

    pcl_ndt_rough_.reset(new NDTType());
    pcl_ndt_rough_->setResolution(5.0);
    pcl_ndt_rough_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    pcl_ndt_rough_->setStepSize(0.1);
    pcl_ndt_rough_->setMaximumIterations(4);
    pcl_ndt_rough_->setNumThreads(4);

    pcl_icp_.reset(new ICPType());
    pcl_icp_->setMaximumIterations(4);
    pcl_icp_->setTransformationEpsilon(0.01);

    LOG(INFO) << "match name is NDT_OMP"
              << ", MaximumIterations is: " << pcl_ndt_->getMaximumIterations();
}

LidarLoc::~LidarLoc() {
    if (update_map_thread_.joinable()) {
        update_map_quit_ = true;
        update_map_thread_.join();
    }

    recover_pose_out_.close();
}

bool LidarLoc::Init(const std::string& config_path) {
    YAML_IO yaml(config_path);
    options_.map_option_.enable_dynamic_polygon_ = yaml.GetValue<bool>("maps", "with_dyn_area");
    options_.map_option_.max_pts_in_dyn_chunk_ = yaml.GetValue<int>("maps", "max_pts_dyn_chunk");
    options_.map_option_.load_map_size_ = yaml.GetValue<int>("maps", "load_map_size");
    options_.map_option_.unload_map_size_ = yaml.GetValue<int>("maps", "unload_map_size");

    options_.update_kf_dis_ = yaml.GetValue<double>("lidar_loc", "update_kf_dis");
    options_.update_lidar_loc_score_ = yaml.GetValue<double>("lidar_loc", "update_lidar_loc_score");
    options_.min_init_confidence_ = yaml.GetValue<float>("lidar_loc", "min_init_confidence");

    // options_.filter_z_min_ = yaml.GetValue<double>("lidar_loc", "filter_z_min");
    // options_.filter_z_max_ = yaml.GetValue<double>("lidar_loc", "filter_z_max");
    // options_.filter_intensity_min_ = yaml.GetValue<double>("lidar_loc", "filter_intensity_min");
    // options_.filter_intensity_max_ = yaml.GetValue<double>("lidar_loc", "filter_intensity_max");
    options_.lidar_loc_odom_th_ = yaml.GetValue<double>("lidar_loc", "lidar_loc_odom_th");

    options_.init_with_fp_ = yaml.GetValue<bool>("lidar_loc", "init_with_fp");
    options_.enable_parking_static_ = yaml.GetValue<bool>("lidar_loc", "enable_parking_static");
    options_.enable_icp_adjust_ = yaml.GetValue<bool>("lidar_loc", "enable_icp_adjust");
    options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
    options_.try_self_extrap_ = yaml.GetValue<bool>("lidar_loc", "try_self_extrap");

    lidar_loc::grid_search_angle_step = yaml.GetValue<double>("lidar_loc", "grid_search_angle_step");
    lidar_loc::grid_search_angle_range = yaml.GetValue<double>("lidar_loc", "grid_search_angle_range");

    /// 个人修改（2026-10-01）：手动初始位姿配套参数（可选键，缺键时用代码默认值 → 旧 yaml 也能跑）
    if (yaml.HasValue("lidar_loc", "manual_init_rough_search")) {
        options_.manual_init_rough_search_ = yaml.GetValue<bool>("lidar_loc", "manual_init_rough_search");
    }
    if (yaml.HasValue("lidar_loc", "manual_init_yaw_search_deg")) {
        options_.manual_init_yaw_search_deg_ = yaml.GetValue<double>("lidar_loc", "manual_init_yaw_search_deg");
    }
    if (yaml.HasValue("lidar_loc", "manual_init_yaw_search_step")) {
        options_.manual_init_yaw_search_step_ = yaml.GetValue<int>("lidar_loc", "manual_init_yaw_search_step");
    }
    if (yaml.HasValue("lidar_loc", "manual_init_refine_iter")) {
        options_.manual_init_refine_iter_ = yaml.GetValue<int>("lidar_loc", "manual_init_refine_iter");
    }

    /// 个人修改（2026-10-02，方案C）：高度约束 / 地图健康度自检（可选键，缺键时用"恢复原注释代码语义"的默认值）
    ///   说明：with_height 以前同时被 loop_closing（后端高度先验）和定位端读取，含义耦合；
    ///   这里给定位端一个独立开关 lidar_loc.height_constraint_enable，
    ///   未配置时回退到 loop_closing.with_height（= 原注释代码的生效条件），保持向后兼容。
    if (yaml.HasValue("lidar_loc", "height_constraint_enable")) {
        options_.height_constraint_enable_ = yaml.GetValue<bool>("lidar_loc", "height_constraint_enable");
    } else {
        options_.height_constraint_enable_ = options_.with_height_;
    }
    if (yaml.HasValue("lidar_loc", "height_constraint_gain")) {
        options_.height_constraint_gain_ = yaml.GetValue<double>("lidar_loc", "height_constraint_gain");
    }
    if (yaml.HasValue("lidar_loc", "height_constraint_tol")) {
        options_.height_constraint_tol_ = yaml.GetValue<double>("lidar_loc", "height_constraint_tol");
    }
    if (yaml.HasValue("lidar_loc", "map_level_check_enable")) {
        options_.map_level_check_enable_ = yaml.GetValue<bool>("lidar_loc", "map_level_check_enable");
    }
    if (yaml.HasValue("lidar_loc", "map_level_warn_deg")) {
        options_.map_level_warn_deg_ = yaml.GetValue<double>("lidar_loc", "map_level_warn_deg");
    }
    if (yaml.HasValue("lidar_loc", "map_level_radius")) {
        options_.map_level_radius_ = yaml.GetValue<double>("lidar_loc", "map_level_radius");
    }
    if (yaml.HasValue("lidar_loc", "force_2d_zero_z")) {
        options_.force_2d_zero_z_ = yaml.GetValue<bool>("lidar_loc", "force_2d_zero_z");
    }

    LOG(INFO) << "[方案C] height constraint: enable=" << options_.height_constraint_enable_
              << " (loop_closing.with_height=" << options_.with_height_ << "), gain=" << options_.height_constraint_gain_
              << ", tol=" << options_.height_constraint_tol_ << " m"
              << (options_.height_constraint_tol_ <= 1e-9 ? "（完全锁定 z = map_height）"
                                                          : "（软皮带：只约束偏离超过 ±tol 的部分）")
              << ", force_2d=" << options_.force_2d_ << ", force_2d_zero_z=" << options_.force_2d_zero_z_;
    LOG(INFO) << "[方案C] map level check: enable=" << options_.map_level_check_enable_
              << ", warn_deg=" << options_.map_level_warn_deg_ << ", radius=" << options_.map_level_radius_ << " m";
    LOG(INFO) << "manual init rough search: " << options_.manual_init_rough_search_
              << ", yaw search: +/-" << options_.manual_init_yaw_search_deg_ << " deg, step "
              << options_.manual_init_yaw_search_step_ << ", refine iter " << options_.manual_init_refine_iter_;

    LOG(INFO) << "min init confidence: " << options_.min_init_confidence_;

    std::string map_policy = yaml.GetValue<std::string>("maps", "dyn_cloud_policy");
    if (map_policy == "short") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::SHORT;
    } else if (map_policy == "long") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::LONG;
    } else if (map_policy == "persistent") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::PERSISTENT;
    }

    options_.map_option_.delete_when_unload_ = yaml.GetValue<bool>("maps", "delete_when_unload");
    options_.map_option_.load_dyn_cloud_ = yaml.GetValue<bool>("maps", "load_dyn_cloud");
    options_.map_option_.save_dyn_when_quit_ = yaml.GetValue<bool>("maps", "save_dyn_when_quit");
    options_.map_option_.save_dyn_when_unload_ = yaml.GetValue<bool>("maps", "save_dyn_when_unload");

    map_ = std::make_shared<TiledMap>(options_.map_option_);
    map_->LoadMapIndex();

    auto fps = map_->GetAllFP();
    if (!fps.empty()) {
        map_->LoadOnPose(fps.front().pose_);
        /// 更新一次地图，保证有初始数据
        UpdateGlobalMap();
    }

    /// load recover pose if exist
    if (PathExists(options_.recover_pose_path_)) {
        std::ifstream fin(options_.recover_pose_path_);
        double data[7] = {0, 0, 0, 0, 0, 0, 1};
        for (int i = 0; i < 7; ++i) {
            fin >> data[i];
        }

        SE3 pose(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2]));
        FunctionalPoint fp_recover;
        fp_recover.name_ = "recover";
        fp_recover.pose_ = pose;
        map_->AddFP(fp_recover);
    }

    update_map_thread_ = std::thread([this]() { LidarLoc::UpdateMapThread(); });

    /// 个人修改（2026-10-02，方案C）：启动时自检地图地面是否水平（地图一旦倾斜，定位 z 必然漂移）
    if (options_.map_level_check_enable_) {
        CheckMapLevel();
    }

    return true;
}

bool LidarLoc::ProcessCloud(CloudPtr cloud_input) {
    assert(cloud_input != nullptr);

    if (cloud_input->empty() || cloud_input->size() < 50) {
        LOG(WARNING) << "loc input is empty or invalid, sz: " << cloud_input->size();
        return false;
    }

    // CloudPtr cloud(new PointCloudType);
    // pcl::VoxelGrid<PointType> voxel;

    // float sz = 0.1;
    // voxel.setLeafSize(sz, sz, sz);
    // voxel.setInputCloud(cloud_input);
    // voxel.filter(*cloud);

    current_scan_ = cloud_input;

    Align(cloud_input);
    return true;
}

NavState LidarLoc::GetState() {
    UL lock_res(result_mutex_);
    NavState ns;
    ns.SetPose(current_abs_pose_);
    ns.timestamp_ = current_timestamp_;
    ns.confidence_ = current_score_;

    UL lock(lo_pose_mutex_);
    if (!lo_pose_queue_.empty()) {
        auto s = lo_pose_queue_.back();
        ns.SetVel(s.GetVel());
    }

    return ns;
}

bool LidarLoc::ProcessDR(const NavState& state) {
    // 未初始化成功的数据不接收
    if (!state.pose_is_ok_) {
        return false;
    }

    // DR数据check
    UL lock(dr_pose_mutex_);
    if (!dr_pose_queue_.empty()) {
        const double last_stamp = dr_pose_queue_.back().timestamp_;
        if (state.timestamp_ < last_stamp) {
            return false;
        }
    }

    dr_pose_queue_.emplace_back(state);
    while (dr_pose_queue_.size() >= 1000) {
        dr_pose_queue_.pop_front();
    }

    return true;
}

bool LidarLoc::ProcessLO(const NavState& state) {
    /// 理论上相对定位是按时间顺序到达的
    UL lock(lo_pose_mutex_);
    if (!lo_pose_queue_.empty()) {
        const double last_stamp = lo_pose_queue_.back().timestamp_;
        if (state.timestamp_ < last_stamp) {
            LOG(WARNING) << "当前相对定位的结果的时间戳应当比上一个时间戳数值大，实际相减得"
                         << state.timestamp_ - last_stamp;
            return false;
        }
    }

    lo_pose_queue_.emplace_back(state);

    while (lo_pose_queue_.size() >= 50) {
        lo_pose_queue_.pop_front();
    }

    if (state.lidar_odom_reliable_ == false) {
        lo_reliable_ = false;
        lo_reliable_cnt_ = 10;
    } else {
        if (state.lidar_odom_reliable_ && lo_reliable_cnt_ > 0) {
            lo_reliable_cnt_--;
        }

        if (lo_reliable_cnt_ == 0) {
            lo_reliable_ = true;
        }
    }

    return true;
}

bool LidarLoc::YawSearch(SE3& pose, double& confidence, CloudPtr input, CloudPtr output) {
    SE3 init_pose = pose;
    auto RPYXYZ = math::SE3ToRollPitchYaw(init_pose);
    double init_yaw = RPYXYZ.yaw;

    confidence = 0;
    bool yaw_search_success = false;

    int step = lidar_loc::grid_search_angle_step;
    double radius = lidar_loc::grid_search_angle_range * constant::kDEG2RAD;
    double angle_search_step = 2 * radius / step;

    std::vector<double> searched_yaw;
    std::vector<double> scores(step);
    std::vector<int> index;
    std::vector<SE3> pose_opti(step);

    for (int i = 0; i < step; ++i) {
        double search_yaw = init_yaw + i * angle_search_step - radius;
        searched_yaw.emplace_back(search_yaw);
        index.emplace_back(i);
    }

    LOG(INFO) << "init yaw: " << init_yaw << ", p: " << RPYXYZ.pitch << ", ro: " << RPYXYZ.roll << ", search from "
              << searched_yaw.front() << " to " << searched_yaw.back();

    /// 粗分辨率
    std::for_each(index.begin(), index.end(), [&](int i) {
        double fitness_score = 0;
        RPYXYZ.yaw = searched_yaw[i];
        SE3 pose_esti = math::XYZRPYToSE3(RPYXYZ);

        Localize(pose_esti, fitness_score, input, output, true);

        scores[i] = fitness_score;
        pose_opti[i] = pose_esti;
    });

    // find best match
    auto best_score_idx = std::max_element(scores.begin(), scores.end()) - scores.begin();
    confidence = scores.at(best_score_idx);
    pose = pose_opti.at(best_score_idx);

    /// 高分辨率
    if (confidence > options_.min_init_confidence_) {
        Localize(pose, confidence, input, output, false);
    }

    if (confidence > options_.min_init_confidence_) {
        LOG(INFO) << "init success, score: " << confidence << ", th=" << options_.min_init_confidence_;
        Eigen::Vector3d suc_translation = pose.translation();
        Eigen::Matrix3d suc_rotation_matrix = pose.rotationMatrix();
        double suc_x = suc_translation.x();
        double suc_y = suc_translation.y();
        double suc_yaw = atan2(suc_rotation_matrix(1, 0), suc_rotation_matrix(0, 0));
        LOG(INFO) << "localization init success, pose: " << suc_x << ", " << suc_y << ", " << suc_yaw
                  << ", conf: " << confidence;
        yaw_search_success = true;
    }

    return yaw_search_success;
}

bool LidarLoc::InitWithFP(CloudPtr input, const SE3& fp_pose, bool wide_seed_search) {
    assert(input != nullptr && !input->empty());

    // 使用功能点的位置进行定位初始化
    double fitness_score;
    SE3 pose_esti = fp_pose;
    CloudPtr output_cloud(new PointCloudType);
    // loc_inited_ = YawSearch(pose_esti, fitness_score, input, output_cloud);

    /// 第一轮：直接以给定种子做细分辨率配准。
    ///   点得准时（车静止或点得准）这就是最优路径，与修改前行为一致。
    loc_inited_ = Localize(pose_esti, fitness_score, input, output_cloud);

    /// 个人修改（2026-10-01，手动初始位姿配套）：
    ///   手动点选（RViz "2D Pose Estimate" / 外部话题）的位姿天然带"点击延迟"误差
    ///   （车在动时约 0.5~2 m；用户的"现在"比正在配准的那一帧新）与朝向误差，
    ///   细分辨率 NDT 只有 4 次迭代、收敛半径小 ⇒ 第一轮常被 min_init_confidence_ 拒掉
    ///   （实测：种子偏 ~1.3 m 时分值 1.4~1.78 < 1.8）。
    ///   此时用【粗分辨率(5m) NDT + 小范围 yaw 搜索】再找一次种子，让代价函数在自己更宽的
    ///   收敛域内先把位置/朝向拉回来，再做细分辨率收敛；最终仍由同一门槛把关，不会降低安全线。
    if (!loc_inited_ && wide_seed_search && options_.manual_init_rough_search_) {
        SE3 coarse_pose = fp_pose;
        double coarse_score = 0;
        if (SearchSeedRough(input, fp_pose, coarse_pose, coarse_score, output_cloud)) {
            SE3 retry_pose = coarse_pose;
            double retry_score = 0;
            bool retry_ok = Localize(retry_pose, retry_score, input, output_cloud);
            if (retry_ok) {
                loc_inited_ = true;
                pose_esti = retry_pose;
                fitness_score = retry_score;
                LOG(INFO) << "manual init: recovered by rough seed search: "
                          << fp_pose.translation().transpose() << " -> " << coarse_pose.translation().transpose()
                          << ", score " << retry_score;
            } else {
                LOG(WARNING) << "manual init: rough seed search did not help (best rough score " << coarse_score
                             << ", final score " << retry_score << ")";
            }
        }
    }

    /// 个人修改（2026-10-01，手动初始位姿配套）：接受后再迭代精修。
    ///   NDT 单次配准只有 4 次迭代，种子偏 1 m 时往往只收敛到"真值附近的局部最优"：
    ///   实测接受时 1.875，而同路段正常跟踪分值是 2.08 —— 该偏差会被后续跟踪继承
    ///   （预测初值总在偏差点附近 ⇒ 每帧都收敛回偏差点，分值系统性偏低）。
    ///   这里以刚得到的结果为初值再迭代若干次（只在分值变好时采用，绝不劣化），
    ///   让手动初值真正落到与跟踪一致的最优解上。仅手动种子路径启用。
    if (loc_inited_ && wide_seed_search && options_.manual_init_refine_iter_ > 0) {
        for (int i = 0; i < options_.manual_init_refine_iter_; ++i) {
            SE3 refined_pose = pose_esti;
            double refined_score = 0;
            /// 注意：此时 loc_inited_ 已被置 true，Localize 不再做门槛判定，纯粹做一次精配准
            Localize(refined_pose, refined_score, input, output_cloud);
            if (refined_score > fitness_score) {
                LOG(INFO) << "manual init: refine iter " << (i + 1) << ": score " << fitness_score << " -> "
                          << refined_score << ", pose " << refined_pose.translation().transpose();
                fitness_score = refined_score;
                pose_esti = refined_pose;
            } else {
                break;  // 已收敛（或没有变好），停止迭代
            }
        }
    }


    if (loc_inited_) {
        current_timestamp_ = math::ToSec(input->header.stamp);
        localization_result_.confidence_ = fitness_score;
        current_abs_pose_ = pose_esti;
        localization_result_.pose_ = pose_esti;
        localization_result_.timestamp_ = current_timestamp_;
        localization_result_.lidar_loc_valid_ = true;
        localization_result_.status_ = LocalizationStatus::GOOD;

        last_abs_pose_set_ = true;
        last_abs_pose_ = pose_esti;

        current_score_ = fitness_score;
        LOG(INFO) << "fitness_score is: " << fitness_score << ", global_pose is: " << fp_pose.translation().transpose();
        LOG(INFO) << " [Loc init pose]: " << last_abs_pose_.translation().transpose();
        map_height_ = fp_pose.translation()[2];
        map_height_valid_ = true;  // 个人修改（方案C）：高度约束的参考高度就绪

        if (current_lo_pose_set_) {
            // 设置上一次的相对定位结果
            last_lo_pose_ = current_lo_pose_;
            last_lo_pose_set_ = true;

            last_dr_pose_ = current_dr_pose_;
            last_dr_pose_set_ = true;
        }

        //  定位成功，则清空失败记录
        fp_init_fail_pose_vec_.clear();
    } else {
        // 添加失败历史记录
        LOG(INFO) << "init failed, score: " << fitness_score;
        fp_init_fail_pose_vec_.emplace_back(fp_pose);
        fp_last_tried_time_ = 1e-6 * static_cast<double>(input->header.stamp);
    }
    return loc_inited_;
}

bool LidarLoc::SearchSeedRough(CloudPtr input, const SE3& seed, SE3& best_pose, double& best_score, CloudPtr out) {
    /// 个人修改（2026-10-01，手动初始位姿配套）：粗分辨率 NDT 上的"种子搜索"。
    ///   与 YawSearch() 的区别：① 只搜种子附近 ±manual_init_yaw_search_deg_（不是 ±180°），
    ///   ② 同时也把【原种子】作为一个候选（用户点得准时它最优），③ 只用于手动初始位姿。
    ///   粗分辨率(5m)的代价函数更平滑、收敛半径更大，因此能吸收手动点选带来的米级位置偏差
    ///   与较大朝向偏差；返回值只是"找到更好的种子"，最终是否初始化成功仍由细分辨率 NDT +
    ///   min_init_confidence_ 判定（见 InitWithFP）。
    if (pcl_ndt_rough_ == nullptr) {
        LOG(WARNING) << "manual init: rough resolution ndt is not ready, keep the given seed";
        return false;
    }

    const int step = std::max(1, options_.manual_init_yaw_search_step_);
    const double radius = options_.manual_init_yaw_search_deg_ * constant::kDEG2RAD;
    const auto seed_rpy = math::SE3ToRollPitchYaw(seed);

    best_score = 0;
    best_pose = seed;

    /// 候选 0：原种子
    {
        SE3 cand = seed;
        double score = 0;
        Localize(cand, score, input, out, true);
        if (score > best_score) {
            best_score = score;
            best_pose = cand;
        }
    }

    /// 候选 1..step：种子 yaw ± radius 内的均匀采样
    for (int i = 0; i < step; ++i) {
        const double offset = (step == 1) ? 0.0 : (-radius + 2.0 * radius * static_cast<double>(i) / (step - 1));
        auto rpy = seed_rpy;
        rpy.yaw = seed_rpy.yaw + offset;

        SE3 cand = math::XYZRPYToSE3(rpy);
        double score = 0;
        Localize(cand, score, input, out, true);
        if (score > best_score) {
            best_score = score;
            best_pose = cand;
        }
    }

    LOG(INFO) << "manual init: rough seed search +/-" << options_.manual_init_yaw_search_deg_ << " deg, " << step
              << " yaw samples, best rough score " << best_score << ", best seed "
              << best_pose.translation().transpose();
    return true;
}

void LidarLoc::ResetLastPose(const SE3& last_pose) {
    last_abs_pose_ = last_pose;

    // TODO：清空动态图层

    return;
}

/**
 * 个人修改（2026-10-02，方案C）：对最终位姿施加"高度约束"（z 拉回首帧功能点高度 map_height_）。
 *
 *   为什么需要：定位端唯一的绝对观测就是点云匹配，匹配的 z 完全由地图决定；
 *   地图一旦倾斜（建图时世界系没和重力对齐），匹配就会忠实地跟着倾斜的地图走，
 *   z 会沿路径持续漂移 —— 这是"定位 z 一直往下掉"在定位端的直接体现。
 *
 *   语义（见 Options 注释）：
 *     tol <= 0 ：z 完全锁定到 map_height_（= 原代码注释里那段 with_height_ 逻辑的行为）
 *     tol >  0 ：软皮带，只把超出 ±tol 的部分拉回来，允许小范围真实起伏
 *     gain    ：修正增益，0 = 关闭
 *   注意：本函数只改 z，不动 x/y/yaw，因此不会影响匹配在水平面内的结果。
 */
void LidarLoc::ApplyHeightConstraint(SE3& pose, const char* tag) {
    if (!options_.height_constraint_enable_ || options_.height_constraint_gain_ <= 1e-9 || !map_height_valid_) {
        return;
    }

    const double z = pose.translation().z();
    const double err = map_height_ - z;

    double applied = err;
    if (options_.height_constraint_tol_ > 1e-9) {
        applied = std::max(-options_.height_constraint_tol_, std::min(options_.height_constraint_tol_, err));
    }

    if (std::fabs(applied) < 1e-9) {
        return;
    }

    Vec3d t = pose.translation();
    t.z() = z + options_.height_constraint_gain_ * applied;
    pose.translation() = t;

    /// 偏差较大时打日志，便于观察约束是否在起作用
    if (std::fabs(err) > 0.05) {
        LOG_EVERY_N(INFO, 20) << "[方案C][height constraint] " << tag << ": z " << z << " -> " << pose.translation().z()
                              << " (map_height = " << map_height_ << ", err = " << err
                              << ", tol = " << options_.height_constraint_tol_ << ")";
    }
}

/**
 * 个人修改（2026-10-02，方案C）：地图地面健康度自检。
 *
 *   做两件事：
 *     ① 用起点附近的地面点拟合一个平面，给出【地面倾角】与【传感器离地高度】；
 *     ② 倾角超过阈值（默认 2°）就打 ERROR，明确指出"地图是倾斜的、定位 z 会漂移、请重建地图"。
 *
 *   为什么放在定位端：本工程的定位结果 z 完全由地图决定，所以"z 一直漂"的第一嫌疑永远是地图本身。
 *   把这一步做成启动自检，就不必每次临时写脚本去量（本次诊断正是用同样的方法量出 22° 的）。
 *
 *   实现要点（对倾斜地图同样有效）：
 *     · 取功能点（优先 start）水平位置 R 半径内的点；
 *     · 在参考点下方 [0.2, 3.0] m 内按 0.1 m 分层统计，取【点最多的一层】作为地面
 *       （地面通常是场景里最密集的水平面）；
 *     · 对该层 ±0.15 m 内的点做平面拟合（math::esti_plane），并统一法向朝上。
 */
void LidarLoc::CheckMapLevel() {
    if (map_ == nullptr) {
        LOG(WARNING) << "[方案C][map level] 地图未初始化，跳过地面健康度自检";
        return;
    }

    auto map_cloud = map_->GetAllMap();
    if (map_cloud == nullptr || map_cloud->empty()) {
        LOG(WARNING) << "[方案C][map level] 已加载地图为空，跳过地面健康度自检";
        return;
    }

    /// 参考点：优先 start 功能点（它就是定位初始化用的位姿），否则用第一个功能点
    auto fps = map_->GetAllFP();
    if (fps.empty()) {
        LOG(WARNING) << "[方案C][map level] 地图没有功能点(index.txt)，跳过地面健康度自检";
        return;
    }

    Vec3d ref = fps.front().pose_.translation();
    std::string ref_name = fps.front().name_;
    for (const auto& fp : fps) {
        if (fp.name_ == "start") {
            ref = fp.pose_.translation();
            ref_name = fp.name_;
            break;
        }
    }

    const double radius = std::max(1.0, options_.map_level_radius_);
    const double r2 = radius * radius;

    /// 1) 半径内点、按 z 分层统计（找到"地面层"）
    constexpr double kBinSize = 0.1;
    const double z_low = ref.z() - 3.0;
    const double z_high = ref.z() - 0.2;
    std::map<int, int> hist;
    int in_radius = 0;

    for (const auto& pt : map_cloud->points) {
        const double dx = pt.x - ref.x();
        const double dy = pt.y - ref.y();
        if (dx * dx + dy * dy > r2) {
            continue;
        }
        in_radius++;
        if (pt.z < z_low || pt.z > z_high) {
            continue;
        }
        hist[static_cast<int>(std::floor((pt.z - z_low) / kBinSize))]++;
    }

    if (hist.empty() || in_radius < 100) {
        LOG(WARNING) << "[方案C][map level] 参考点(" << ref_name << ")附近点太少（半径内 " << in_radius
                     << " 个点），无法自检地面";
        return;
    }

    int best_bin = hist.begin()->first;
    int best_cnt = -1;
    for (const auto& kv : hist) {
        if (kv.second > best_cnt) {
            best_cnt = kv.second;
            best_bin = kv.first;
        }
    }

    const double z_band = z_low + (best_bin + 0.5) * kBinSize;

    /// 2) 地面层 ±0.15 m 内做平面拟合
    PointVector ground_pts;
    ground_pts.reserve(best_cnt);
    for (const auto& pt : map_cloud->points) {
        const double dx = pt.x - ref.x();
        const double dy = pt.y - ref.y();
        if (dx * dx + dy * dy > r2) {
            continue;
        }
        if (std::fabs(pt.z - z_band) > 0.15) {
            continue;
        }
        ground_pts.emplace_back(pt);
    }

    if (ground_pts.size() < static_cast<size_t>(fasterlio::MIN_NUM_MATCH_POINTS)) {
        LOG(WARNING) << "[方案C][map level] 地面层点数不足（" << ground_pts.size() << "），无法拟合地面";
        return;
    }

    Vec4f coef;
    if (!math::esti_plane(coef, ground_pts, 0.3f)) {
        LOG(WARNING) << "[方案C][map level] 地面平面拟合失败（点数 " << ground_pts.size() << "）";
        return;
    }

    Vec4d plane = coef.cast<double>();
    if (plane.z() < 0) {
        plane = -plane;  // 统一法向朝上
    }
    const double n_norm = plane.head<3>().norm();
    if (n_norm < 1e-6) {
        return;
    }
    plane /= n_norm;

    const double cos_tilt = std::min(1.0, std::max(-1.0, plane.z()));
    const double tilt_deg = std::acos(cos_tilt) * 180.0 / M_PI;

    const double a = plane(0), b = plane(1), c = plane(2), d = plane(3);
    const double z_ground_at_ref = (std::fabs(c) > 1e-6) ? -(a * ref.x() + b * ref.y() + d) / c : 0.0;
    const double sensor_height = ref.z() - z_ground_at_ref;
    const double slope = std::tan(tilt_deg * M_PI / 180.0);

    map_ground_plane_ = plane;
    map_ground_tilt_deg_ = tilt_deg;

    LOG(INFO) << "[方案C][map level] 地图地面拟合: 参考点 " << ref_name << " = " << ref.transpose() << ", 半径 "
              << radius << " m 内 " << in_radius << " 点, 地面层 z = " << z_band << " (" << ground_pts.size()
              << " 点), 平面 [a,b,c,d] = [" << a << ", " << b << ", " << c << ", " << d << "]";
    LOG(INFO) << "[方案C][map level] 地面倾角 = " << tilt_deg << " deg (斜率 " << slope
              << " m/m), 参考点处传感器离地 = " << sensor_height << " m";

    if (tilt_deg > options_.map_level_warn_deg_) {
        LOG(ERROR) << "[方案C][map level] 地图地面不水平（倾角 " << tilt_deg << " deg > " << options_.map_level_warn_deg_
                   << " deg）：说明【建图时世界系没有和重力对齐】，轨迹与地图被整体刚性倾斜。"
                   << "后果：定位端匹配只能忠实跟随这张倾斜地图 ⇒ z 沿路径持续漂移（" << slope
                   << " m/m，每走 10 m 掉 " << slope * 10.0
                   << " m），2.5D 栅格图切层与依赖 z 常数的下游模块也会失效。"
                   << "★定位端无法通过匹配修正它，请用修好的建图端【重建地图】（见 doc/pose_gravity_zdrift_fix_report.md）。"
                   << "临时缓解：lidar_loc.height_constraint_enable=true + height_constraint_tol=0（锁 z）。";
    } else {
        LOG(INFO) << "[方案C][map level] 地图地面基本水平（倾角 " << tilt_deg << " deg <= " << options_.map_level_warn_deg_
                  << " deg），定位 z 不会被地图带偏";
    }
}

bool LidarLoc::TryOtherSolution(CloudPtr input, SE3& pose) {
    double fitness_score;
    SE3 pose_esti = pose;
    CloudPtr output_cloud(new PointCloudType);

    bool loc_success = Localize(pose_esti, fitness_score, input, output_cloud);

    if (loc_success) {
        // 激光重置逻辑
        float score_th = std::min(1.5 * current_score_, current_score_ + 0.3);
        if (fitness_score > score_th && fitness_score > 1.0) {
            // 显著好于现在的估计
            LOG(WARNING) << "rtk solution is significantly better: " << fitness_score << " " << current_score_;
            pose = pose_esti;
            localization_result_.lidar_loc_smooth_flag_ = false;
            return true;
        } else {
            LOG(INFO) << "not using rtk solution: " << fitness_score << " " << current_score_;
            return false;
        }
    }
    return false;
}

bool LidarLoc::UpdateGlobalMap() {
    NDTType::Ptr ndt(new NDTType());
    ndt->setResolution(1.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setStepSize(0.1);
    ndt->setMaximumIterations(4);
    ndt->setNumThreads(4);

    map_->SetNewTargetForNDT(ndt);
    ndt->initCompute();

    UL lock(match_mutex_);
    pcl_ndt_ = ndt;

    if (!loc_inited_) {
        NDTType::Ptr ndt_rough(new NDTType());
        ndt_rough->setResolution(5.0);
        ndt_rough->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt_rough->setStepSize(0.1);
        ndt_rough->setMaximumIterations(4);
        ndt_rough->setNumThreads(4);

        map_->SetNewTargetForNDT(ndt_rough);
        // ndt_rough->initCompute();

        pcl_ndt_rough_ = ndt_rough;
    }

    if (options_.enable_icp_adjust_) {
        ICPType::Ptr icp(new ICPType());
        CloudPtr map_cloud(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel;
        auto sz = 0.5;
        voxel.setLeafSize(sz, sz, sz);
        voxel.setInputCloud(map_->GetAllMap());
        voxel.filter(*map_cloud);
        icp->setInputTarget(map_cloud);
        icp->setMaximumIterations(4);
        icp->setTransformationEpsilon(0.01);
        pcl_icp_ = icp;
    }

    return true;
}

void LidarLoc::UpdateMapThread() {
    LOG(INFO) << "UpdateMapThread thread is running";
    while (!update_map_quit_) {
        if (map_->MapUpdated() || map_->DynamicMapUpdated()) {
            UpdateGlobalMap();

            if (ui_) {
                ui_->UpdatePointCloudGlobal(map_->GetStaticCloud());
                ui_->UpdatePointCloudDynamic(map_->GetDynamicCloud());
            }

            map_->CleanMapUpdate();
        }
        usleep(10000);
    }
}

void LidarLoc::SetInitialPose(SE3 init_pose) {
    UL lock(initial_pose_mutex_);
    loc_inited_ = false;
    // map_->ClearMap();

    initial_pose_set_ = true;
    initial_pose_ = init_pose;
    LOG(INFO) << "Set initial pose is: " << initial_pose_.translation().transpose();
}

void LidarLoc::Align(const CloudPtr& input) {
    // 输入必须非空
    assert(input != nullptr);

    // 点云去畸变定到了结束时间，所以该点云的定位也是到结束时间的
    double current_time = math::ToSec(input->header.stamp) + lo::lidar_time_interval;
    current_timestamp_ = current_time;

    LOG(INFO) << "current time: " << std::fixed << std::setprecision(12) << current_timestamp_;

    /// 设置当前帧对应的rel_pose
    if (!AssignLOPose(current_time)) {
        LOG(WARNING) << "assign LO pose failed";
    }

    if (!AssignDRPose(current_time)) {
        LOG(WARNING) << "assign DR pose failed";
    }

    /// 1. 车辆静止处理
    if (parking_ && loc_inited_) {
        LOG(INFO) << "车辆静止，不做匹配";

        UpdateState(input);
        current_abs_pose_ = last_abs_pose_;
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);

        UL lock(result_mutex_);
        localization_result_.timestamp_ = current_time;
        localization_result_.pose_ = current_abs_pose_;

        if (options_.enable_parking_static_) {
            localization_result_.is_parking_ = true;
            localization_result_.valid_ = true;
        }
        return;
    }

    /// 2. 初始化处理
    if (!loc_inited_) {
        UL lock_init(initial_pose_mutex_);
        LOG(INFO) << "initing lidarloc";
        SetInitRltState();

        if (initial_pose_set_) {
            /// 尝试在给定点初始化
            /// 个人修改（2026-10-01，手动初始位姿配套）：
            ///   ① 先 map_->LoadOnPose(initial_pose_)：下面的功能点循环每试一个候选都会加载对应地图块，
            ///      而手动分支原来【漏了这一步】⇒ NDT 目标仍是"上一处加载的地图块"（例如功能点 start 附近），
            ///      远端手动点选时目标点云覆盖不足、分值被明显压低（实测同路段 2.06 → 1.73），
            ///      既可能被门槛误拒，即使成功也会让后续跟踪分值偏低。这里补齐与功能点循环一致的动作。
            ///      （地图块真正的 NDT 目标更新由地图线程完成，若本帧尚未更新，下一帧会带着
            ///       已更新的目标重试 —— initial_pose_set_ 在成功前一直为 true。）
            ///   ② 传 wide_seed_search = true：手动点选带点击延迟/朝向误差，先用粗分辨率 NDT
            ///      小范围搜索一遍再精配准（普通功能点初始化仍走原路径）。
            map_->LoadOnPose(initial_pose_);
            if (InitWithFP(input, initial_pose_, true)) {
                LOG(INFO) << "init with external pose: " << initial_pose_.translation().transpose();
                initial_pose_set_ = false;
                return;
            }
        }

        if (options_.init_with_fp_) {
            /// 从功能点初始化
            /// 如果之前尝试过，那么需要间隔一段时间再进行搜索
            if (!fp_init_fail_pose_vec_.empty() && current_dr_pose_set_) {
                SE3 last_tried_pose = fp_init_fail_pose_vec_.back();
                bool should_try =
                    (current_time - fp_last_tried_time_) > 2.0 ||
                    (current_dr_pose_.translation() - last_tried_pose.translation()).norm() > 0.3 ||
                    (current_dr_pose_.so3().inverse() * last_tried_pose.so3()).log().norm() > 10 * M_PI / 180.0;
                if (!should_try) {
                    LOG(INFO) << "skip trying init, please move to another place.";
                    return;
                }
            } else {
                LOG(INFO) << "fp tried pose: " << fp_init_fail_pose_vec_.size()
                          << ", dr pose set: " << current_dr_pose_set_;
            }

            auto all_fps = map_->GetAllFP();
            bool fp_init_success = false;
            for (const auto& fp : all_fps) {
                map_->LoadOnPose(fp.pose_);
                if (InitWithFP(input, fp.pose_)) {
                    LOG(INFO) << "init with fp: " << fp.name_;
                    fp_init_success = true;
                    break;
                }
            }

            if (!fp_init_success) {
                LOG(INFO) << "FP init failed.";
                if (current_dr_pose_set_) {
                    LOG(INFO) << "record fp failed time: " << std::setprecision(12) << current_time
                              << ", pose: " << current_dr_pose_.translation().transpose();
                    fp_last_tried_time_ = current_time;
                    fp_init_fail_pose_vec_.emplace_back(current_dr_pose_);
                }
            } else {
                fp_last_tried_time_ = 0;
                fp_init_fail_pose_vec_.clear();
            }
        }

        /// 初始化未成功时，不往下走流程
        return;
    }

    /// 4. 设置当前帧对应的 pose guess
    /// NOTE: LO设置预测的位置和LidarLoc自身递推设置预测的方法并不完全一致，自身外推容易受噪声影响

    SE3 guess_from_lo = last_abs_pose_;
    if (last_lo_pose_set_ && current_lo_pose_set_) {
        // 如果有里程计，则用两个时刻的相对定位来递推，估计一个当前pose的初值
        const SE3 delta = last_lo_pose_.inverse() * current_lo_pose_;
        guess_from_lo = last_abs_pose_ * delta;

        LOG(INFO) << "current lo pose: " << current_lo_pose_.translation().transpose();
        LOG(INFO) << "last lo pose: " << last_lo_pose_.translation().transpose();
        LOG(INFO) << "lo motion: " << delta.translation().transpose();
        LOG(INFO) << "last abs pose: " << last_abs_pose_.translation().transpose();
        // guess_from_lo.translation()[2] = 0;
        LOG(INFO) << "loc using lo guess: " << guess_from_lo.translation().transpose();
    }

    SE3 guess_from_self = guess_from_lo;
    if (lidar_loc_pose_queue_.size() >= 2) {
        SE3 pred;
        TimedPose match;
        if (math::PoseInterp<TimedPose>(
                current_time, lidar_loc_pose_queue_, [](const TimedPose& p) { return p.timestamp_; },
                [](const TimedPose& p) { return p.pose_; }, pred, match, 2.0)) {
            guess_from_self = pred;
        }
    }

    // SE3 guess_from_dr = guess_from_lo;
    // if (last_dr_pose_set_ && current_dr_pose_set_) {
    //     const SE3 delta = last_dr_pose_.inverse() * current_dr_pose_;
    //     guess_from_dr = last_abs_pose_ * delta;
    //     // guess_from_dr.translation()[2] = 0;
    // }

    // bool try_dr = false;
    // if (((guess_from_dr.translation() - guess_from_lo.translation()).norm() >= try_other_guess_trans_th_ ||
    //      (guess_from_dr.so3().inverse() * guess_from_lo.so3()).log().norm() >= try_other_guess_rot_th_)) {
    //     LOG(INFO) << "trying dr pose: " << guess_from_dr.translation().transpose() << ", "
    //               << (guess_from_dr.so3().inverse() * guess_from_lo.so3()).log().norm()
    //               << ", vel_norm: " << current_vel_b_.norm();
    //     try_dr = true;
    // }

    bool try_self = false;
    // if (options_.try_self_extrap_) {
    //     if (((guess_from_self.translation() - guess_from_lo.translation()).norm() >= try_other_guess_trans_th_ ||
    //          (guess_from_self.so3().inverse() * guess_from_lo.so3()).log().norm() >= try_other_guess_rot_th_) &&
    //         ((guess_from_dr.translation() - guess_from_self.translation()).norm() >= try_other_guess_trans_th_ ||
    //          (guess_from_dr.so3().inverse() * guess_from_self.so3()).log().norm() >= try_other_guess_rot_th_)) {
    //         LOG(INFO) << "trying self extrap pose: " << guess_from_self.translation().transpose() << ", "
    //                   << (guess_from_self.so3().inverse() * guess_from_lo.so3()).log().norm();
    //         try_self = true;
    //     }
    // }

    /// 5. 载入地图, 与地图匹配定位
    /// 尝试各种初始估计
    CloudPtr output_cloud(new PointCloudType);
    double fitness_score = 0;
    SE3 current_pose_esti = guess_from_lo;
    bool loc_success_lo, loc_success_self, loc_success_dr;
    loc_success_lo = loc_success_self = loc_success_dr = false;
    bool loc_success = false;

    /// 注意load on pose存在滞后，优先load on DR
    map_->LoadOnPose(guess_from_lo);

    loc_success_lo = Localize(current_pose_esti, fitness_score, input, output_cloud);  // LO 那个肯定会算
    double score_lo = fitness_score;

    SE3 res_of_lo = current_pose_esti;
    SE3 res_of_dr = current_pose_esti;
    SE3 res_of_self = current_pose_esti;
    double score_dr = 0;

    // 先尝试外部预测，最后用自身
    // if (try_dr) {
    //     /// 尝试DR外推的pose
    //     res_of_dr = guess_from_dr;
    //     loc_success_dr = Localize(res_of_dr, score_dr, input, output_cloud);
    //     if (score_dr > (fitness_score - 0.1)) {
    //         current_pose_esti = res_of_dr;
    //         fitness_score = score_dr;
    //         LOG(INFO) << "take dr guess: " << current_pose_esti.translation().transpose()
    //                   << " , confidence: " << score_dr << ", v_norm: " << current_vel_b_.norm();
    //     }
    // }

    // 用纯激光定位有点太抖了，加一些权重
    Vec6d delta = (guess_from_lo.inverse() * current_pose_esti).log();
    SE3 esti_balanced = guess_from_lo * SE3::exp(delta * 0.1);
    current_pose_esti = esti_balanced;

    // double score_self = 0;
    // if (try_self) {
    //     /// 尝试自身外推的pose
    //     LOG(INFO) << "localize with extrap";

    //     res_of_self = guess_from_self;
    //     loc_success_self = Localize(res_of_self, score_self, input, output_cloud);

    //     // 避免分值接近但长时间采信自身预测，此处更相信外部预测源
    //     if (score_self > (fitness_score + 0.1)) {
    //         current_pose_esti = res_of_self;
    //         fitness_score = score_self;
    //         LOG(INFO) << "take self guess: " << current_pose_esti.translation().transpose()
    //                   << " , confidence: " << score_self;
    //     }
    // }

    /// NOTE 如果LO, DR出发点和收敛点不同，但分值相近，说明场景可能处在退化状态，此时使用DR预测的Pose
    // if (try_dr && (res_of_lo.translation() - res_of_dr.translation()).head<2>().norm() > 0.2 &&
    //     fabs(score_lo - score_dr) < 0.2 && score_lo < 1.2) {
    //     LOG(WARNING) << "判定激光定位进入退化状态，现在会使用DR递推pose而不是激光定位位置";
    //     current_pose_esti = guess_from_dr;
    // }

    if (options_.force_2d_) {
        PoseRPYD RPYXYZ = math::SE3ToRollPitchYaw(current_pose_esti);
        /// 个人修改（2026-10-02，方案C）：force_2d 的语义修正
        ///   原实现连 z 一起归零（RPYXYZ.z = 0），那是"2D 导航、地图原点在地面"的布局假设；
        ///   本平台地图原点在 IMU/雷达处、地面在 z≈-0.68 m，且路线可能有坡度 ⇒ z 归零会让定位整体偏低。
        ///   现在默认只约束 roll/pitch（"车是水平的"这条物理事实），z 交给 height_constraint 处理；
        ///   需要老行为时把 lidar_loc.force_2d_zero_z 设为 true。
        ///   另外记录一下"匹配结果相对水平面偏了多少"，便于发现地图倾斜/匹配异常。
        const double raw_roll_deg = RPYXYZ.roll * 180.0 / M_PI;
        const double raw_pitch_deg = RPYXYZ.pitch * 180.0 / M_PI;
        if (std::fabs(raw_roll_deg) > 5.0 || std::fabs(raw_pitch_deg) > 5.0) {
            LOG_EVERY_N(WARNING, 20) << "[方案C] 匹配位姿明显不水平: roll = " << raw_roll_deg
                                     << " deg, pitch = " << raw_pitch_deg
                                     << " deg（>5°）—— 常见原因是【地图本身倾斜】或匹配退化；"
                                     << "force_2d 已把姿态约束为水平，但 z 仍会跟随倾斜地图漂移";
        }

        RPYXYZ.roll = 0;
        RPYXYZ.pitch = 0;
        if (options_.force_2d_zero_z_) {
            RPYXYZ.z = 0;
        }
        current_pose_esti = math::XYZRPYToSE3(RPYXYZ);
    }

    /// 个人修改（2026-10-02，方案C）：恢复并规范化被注释掉的 with_height_ 高度约束
    ///   （见 ApplyHeightConstraint 与 LidarLoc::Options 的说明：tol=0 即原注释代码的"锁 z"行为）
    ApplyHeightConstraint(current_pose_esti, "match");

    current_abs_pose_ = current_pose_esti;
    current_score_ = fitness_score;
    double delta_rel_abs_pose = 0;
    bool lidar_loc_odom_valid = true;

    if (loc_success_lo || loc_success_self || loc_success_dr) {
        loc_success = true;
    } else {
        LOG(INFO) << "loc success is false.";
    }

    if (loc_success) {
        lidar_loc_odom_valid = CheckLidarOdomValid(current_pose_esti, delta_rel_abs_pose);
        last_timestamp_ = current_timestamp_;  // 成功时，更新上一时刻激光定位时间
        match_fail_count_ = 0;
    } else {
        current_score_ = fitness_score;
        LOG(WARNING) << "localization failed! score: " << current_score_;

        ///  若连续3帧匹配失败就设一个大分值
        ++match_fail_count_;
    }

    /// 确定激光定位是否满足平滑性要求
    Vec3d dpred = current_abs_pose_.translation() - guess_from_self.translation();
    if (fabs(dpred[0]) < 0.5 && fabs(dpred[1]) < 0.5 &&
        (current_abs_pose_.so3().inverse() * guess_from_self.so3()).log().norm() < 2.0 * M_PI / 180.0) {
        localization_result_.lidar_loc_smooth_flag_ = true;
    } else {
        localization_result_.lidar_loc_smooth_flag_ = false;
    }

    localization_result_.lidar_loc_odom_reliable_ = lo_reliable_;
    localization_result_.is_parking_ = false;

    // if (ui_) {
    //     ui_->UpdatePredictPose(guess_from_lo);
    // }

    /// 7. 输出结果
    {
        UL lock(result_mutex_);
        localization_result_.timestamp_ = current_timestamp_;
        localization_result_.confidence_ = fitness_score;
        if (match_fail_count_ < 100) {
            localization_result_.lidar_loc_valid_ = true;
            localization_result_.status_ = LocalizationStatus::GOOD;
        } else if (match_fail_count_ >= 100 && match_fail_count_ < 300) {
            localization_result_.lidar_loc_valid_ = false;
            localization_result_.status_ = LocalizationStatus::FOLLOWING_DR;
        } else {
            match_fail_count_ = 300;
            localization_result_.lidar_loc_valid_ = false;
            localization_result_.status_ = LocalizationStatus::FAIL;
        }

        localization_result_.lidar_loc_odom_delta_ = delta_rel_abs_pose;
        localization_result_.lidar_loc_odom_error_normal_ = lidar_loc_odom_valid;
        localization_result_.pose_ = current_pose_esti;
    }

    UpdateState(input);

    /// 8. 更新动态图层
    /// 条件：1. 定位成功 2. 与上次更新间隔一定距离 3. RTK与激光定位横纵向误差都小于0.3，或者匹配分值大于1.0
    bool score_cond = current_score_ > options_.update_lidar_loc_score_;

    if (options_.update_dynamic_cloud_ && loc_success &&
        (((current_pose_esti.translation() - last_dyn_upd_pose_.pose_.translation()).norm() >
          options_.update_kf_dis_) ||
         fabs(current_time - last_dyn_upd_pose_.timestamp_) > options_.update_kf_time_)) {
        if (score_cond /*  || update_cache_dis_ < options_.max_update_cache_dis_ */) {
            // LOG(INFO) << "passing through z filter, input:" << input->size();
            pcl::PassThrough<PointType> pass;
            pass.setInputCloud(input);

            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.5, options_.filter_z_max_);

            CloudPtr input_z_filter(new PointCloudType());
            pass.filter(*input_z_filter);

            if (!input_z_filter->empty()) {
                CloudPtr cloud_t(new PointCloudType());
                pcl::transformPointCloud(*input_z_filter, *cloud_t, current_pose_esti.matrix());

                // 以现在的scan来更新地图
                map_->UpdateDynamicCloud(cloud_t, true);

                last_dyn_upd_pose_.timestamp_ = current_time;
                last_dyn_upd_pose_.pose_ = current_pose_esti;
            }
        }
    }

    if (lidar_loc_pose_queue_.empty()) {
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
    } else if (current_time > lidar_loc_pose_queue_.back().timestamp_) {
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
    }

    while (lidar_loc_pose_queue_.size() > 1000) {
        lidar_loc_pose_queue_.pop_front();
    }

    ave_scores_.emplace_back(current_score_);
    while (ave_scores_.size() > 20) {
        ave_scores_.pop_front();
    }

    /// 9. save for recover pose
    recover_pose_out_.open(options_.recover_pose_path_);
    if (recover_pose_out_) {
        Vec3d t = current_pose_esti.translation();
        Quatd q = current_pose_esti.unit_quaternion();
        recover_pose_out_ << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " "
                          << q.w();
        recover_pose_out_.close();
    }
}

bool LidarLoc::CheckLidarOdomValid(const SE3& current_pose_esti, double& delta_posi) {
    delta_posi = ((last_lo_pose_.inverse() * current_lo_pose_).translation() -
                  (last_abs_pose_.inverse() * current_pose_esti).translation())
                     .head(2)
                     .norm();

    bool valid = true;

    if (delta_posi > options_.lidar_loc_odom_th_ && last_lo_pose_set_) {
        LOG(INFO) << "delta_rel_abs_pose is: " << delta_posi;
        LOG(INFO) << "LO相对pose: " << last_lo_pose_.translation().transpose() << " "
                  << current_lo_pose_.translation().transpose() << " "
                  << (last_lo_pose_.inverse() * current_lo_pose_).translation().transpose();
        LOG(INFO) << "Lidar Loc计算相对pose: " << last_abs_pose_.translation().transpose() << " "
                  << current_pose_esti.translation().transpose() << " "
                  << (last_abs_pose_.inverse() * current_pose_esti).translation().transpose();
        lo_reliable_ = false;
        lo_reliable_cnt_ = 10;
        valid = false;
    }

    last_abs_pose_ = current_pose_esti;
    last_lo_pose_ = current_lo_pose_;
    last_lo_pose_set_ = true;
    last_dr_pose_ = current_dr_pose_;
    last_dr_pose_set_ = true;

    return valid;
}

bool LidarLoc::Localize(SE3& pose, double& confidence, CloudPtr input, CloudPtr output, bool use_rough_res) {
    Eigen::Matrix4f trans;
    bool loc_success = false;
    Eigen::Matrix4f guess_pose = pose.matrix().cast<float>();

    LOG(INFO) << "loc from: " << pose.translation().transpose();

    if (pcl_ndt_->getInputTarget() == nullptr) {
        LOG(INFO) << "lidar loc target is null, skip";
        return false;
    }

    UL lock(match_mutex_);
    NDTType::Ptr ndt = nullptr;

    if (use_rough_res) {
        ndt = pcl_ndt_rough_;
    } else {
        ndt = pcl_ndt_;
    }

    ndt->setInputSource(input);
    ndt->align(*output, guess_pose);
    trans = ndt->getFinalTransformation();
    confidence = ndt->getTransformationProbability();

    auto tgt = ndt->getInputTarget();
    if (!tgt->empty()) {
        pcl::io::savePCDFile("./data/tgt.pcd", *tgt);
    }

    // 个人修改：此处原为【恒真】分支（if/else 两个分支都赋 loc_success = true），
    //   使"初始化最小匹配分值" min_init_confidence_ 完全失效：任何初始化候选
    //   （外部初始位姿 / 功能点）即使配准分值极低，也会被当作"初始化成功"，
    //   InitWithFP 随即把 loc_inited_ 置 true，于是
    //     ① 功能点回退分支（index.txt 的 start，分值通常 >2.0）永远不会被执行；
    //     ② 之后再也没有重新初始化的机会（重试逻辑全部在 !loc_inited_ 分支内），
    //        定位会带着这个错误初值一路漂移，表现为"定位完全不可用"。
    //   现在按原设计意图把关：仅"尚未初始化"时用分值判定，跟踪阶段（loc_inited_ == true）
    //   行为保持不变，不影响正常跟踪。
    if (loc_inited_ == false) {
        loc_success = (confidence > options_.min_init_confidence_);
        if (!loc_success) {
            LOG(WARNING) << "init confidence too low: " << confidence << " <= " << options_.min_init_confidence_
                         << ", reject this init candidate";
        }
    } else {
        loc_success = true;
    }

    if (options_.enable_icp_adjust_ && loc_inited_) {
        Eigen::Matrix4f adjust_trans;
        CloudPtr input_voxel(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel_icp;

        double ls = 0.2;
        voxel_icp.setLeafSize(ls, ls, ls);
        voxel_icp.setInputCloud(input);
        voxel_icp.filter(*input_voxel);
        pcl_icp_->setInputSource(input_voxel);
        Timer::Evaluate([&]() { pcl_icp_->align(*output, trans); }, "pcl_icp adjust", true);
        adjust_trans = pcl_icp_->getFinalTransformation();

        Eigen::Matrix3f rotation_diff = trans.block<3, 3>(0, 0).transpose() * adjust_trans.block<3, 3>(0, 0);
        Eigen::AngleAxisf angle_axis(rotation_diff);
        float a = angle_axis.angle();
        float d = (trans.block<3, 1>(0, 3) - adjust_trans.block<3, 1>(0, 3)).norm();
        LOG(INFO) << "icp adjust d: " << d << ", a: " << a;

        if (pcl_icp_->hasConverged() && std::fabs(d) <= 0.05 && std::fabs(a) <= 0.05) {
            LOG(INFO) << "icp ajust trans set success";
            trans = adjust_trans;
        }
    }

    Eigen::Matrix3d rot = trans.block<3, 3>(0, 0).cast<double>();
    Quatd q_3d = Quatd(rot);
    Vec3d t_3d = trans.block<3, 1>(0, 3).cast<double>();
    q_3d.normalize();
    pose = SE3(q_3d, t_3d);

    LOG(INFO) << "confidence: " << confidence << ", t: " << t_3d.transpose() << ", succ: " << loc_success;

    return loc_success;
}

bool LidarLoc::CheckStatic(double timestamp) {
    if (parking_) {
        // if (current_vel_b_.norm() < common::options::lo::parking_speed) {
        // LOG(INFO) << "car is in static mode";
        static_count_++;
        if (static_count_ >= lo::parking_count) {
            static_count_ = 0;
            return false;
        }
        return true;
    } else {
        static_count_ = 0;
        return false;
    }
}

void LidarLoc::UpdateState(const CloudPtr& input) { last_timestamp_ = current_timestamp_; }

void LidarLoc::SetInitRltState() {
    UL lock(result_mutex_);
    localization_result_.confidence_ = 0.0;
    localization_result_.timestamp_ = current_timestamp_;
    localization_result_.lidar_loc_valid_ = false;
    localization_result_.status_ = LocalizationStatus::INITIALIZING;
}

bool LidarLoc::LocInited() {
    // UL lock( data_mutex_);
    return loc_inited_;
}

bool LidarLoc::AssignLOPose(double timestamp) {
    UL lock(lo_pose_mutex_);
    SE3 interp_pose;
    NavState best_match;
    // 无法拿到最新的，插值结果都是外推出来的，和真实有时偏差会很大（╯﹏╰）
    // if (!lo_pose_queue_.empty()) {
    //     LOG(INFO) << "lo interp: " << timestamp << " " << lo_pose_queue_.back().timestamp_ << " "
    //               << lo_pose_queue_.back().pose_.translation().transpose();
    // }

    bool pose_interp_success = math::PoseInterp<NavState>(
        timestamp, lo_pose_queue_, [](const NavState& dr) { return dr.timestamp_; },
        [](const NavState& dr) { return dr.GetPose(); }, interp_pose, best_match, 5.0);

    if (pose_interp_success) {
        current_lo_pose_ = interp_pose;
        current_lo_pose_set_ = true;

        current_vel_b_ = best_match.GetRot().inverse() * best_match.GetVel();
        current_vel_ = best_match.GetVel();

        /// 个人修改（2026-10-02，方案C）：恢复被注释掉的 with_height_ 处理。
        ///   这里 current_lo_pose_ 是"LO 在该时刻的世界位姿"，它只被用作【相对量】
        ///   （delta = last_lo_pose_⁻¹ * current_lo_pose_）来外推 guess：
        ///   把它的 z 归到 map_height_ 上，等价于【让 LO 不再向 guess 里注入 z 变化】，
        ///   即只让 LO 提供平面内的 x/y/yaw 递推 ⇒ 倾斜的 LIO 不会再带着 guess 一起往下掉。
        if (options_.height_constraint_enable_ && options_.height_constraint_gain_ > 1e-9 && map_height_valid_) {
            current_lo_pose_.translation()[2] = map_height_;
        }

        return true;
    } else {
        current_lo_pose_set_ = false;
        return false;
    }
}

bool LidarLoc::AssignDRPose(double timestamp) {
    UL lock(dr_pose_mutex_);
    SE3 interp_pose;
    NavState best_match;
    bool pose_interp_success = math::PoseInterp<NavState>(
        timestamp, dr_pose_queue_, [](const NavState& dr) { return dr.timestamp_; },
        [](const NavState& dr) { return dr.GetPose(); }, interp_pose, best_match, 5.0);

    if (pose_interp_success) {
        parking_ = best_match.is_parking_;
        current_dr_pose_ = interp_pose;
        current_dr_pose_set_ = true;

        /// 个人修改（2026-10-02，方案C）：同 AssignLOPose —— DR 也只提供平面内递推，
        ///   不把 z 变化注入 guess（原注释代码的语义，已用 map_height_valid_ 做有效性保护）
        if (options_.height_constraint_enable_ && options_.height_constraint_gain_ > 1e-9 && map_height_valid_) {
            current_dr_pose_.translation()[2] = map_height_;
        }

        return true;
    } else {
        parking_ = false;
        current_dr_pose_set_ = false;
        return false;
    }
}

void LidarLoc::Finish() {
    if (map_) {
        LOG(INFO) << "saving maps";
        update_map_quit_ = true;
        update_map_thread_.join();

        /// 永久保存时，再存储地图
        if (options_.map_option_.policy_ == TiledMap::DynamicCloudPolicy::PERSISTENT &&
            options_.map_option_.save_dyn_when_quit_ && !has_set_pose_) {
            map_->SaveToBin(true);
            LOG(INFO) << "dynamic maps saved";
        }
    }
}

}  // namespace lightning::loc
