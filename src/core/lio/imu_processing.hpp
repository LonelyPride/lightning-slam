#pragma once

#ifndef FASTER_LIO_IMU_PROCESSING_H
#define FASTER_LIO_IMU_PROCESSING_H

#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "common/eigen_types.h"
#include "common/measure_group.h"
#include "common/point_def.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_filter.h"
#include "core/lio/pose6d.h"
#include "utils/timer.h"

namespace lightning {

/// IMU处理类
class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void SetExtrinsic(const Vec3d &transl, const Mat3d &rot);
    void SetGyrCov(const Vec3d &scaler);
    void SetAccCov(const Vec3d &scaler);
    void SetGyrBiasCov(const Vec3d &b_g);
    void SetAccBiasCov(const Vec3d &b_a);

    void Process(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &scan);

    bool IsIMUInited() const { return imu_need_init_ == false; }
    void SetUseIMUFilter(bool b) { use_imu_filter_ = b; }

    double GetMeanAccNorm() const { return mean_acc_.norm(); }

    /// 个人修改（方案A+）：静止初始化窗口内的平均比力（IMU 系）。静止时它测的就是"上"方向，
    ///   laser_mapping.cc 用它在本平台 auto 模式下自动确定 IMU→车辆 的安装旋转（对安装倾斜自动适应），
    ///   与 IMUInit() 内部 init_state.rot_ 的重力对齐用的是同一个量。
    Vec3d GetMeanAcc() const { return mean_acc_; }

    // ---------------------------------------------------------------------------------------------
    //  个人修改（2026-10-02，方案C：姿态/重力根治）——对外暴露"当前比力/角速度"与初始化自检量
    //    供 laser_mapping.cc 的"重力作为加计观测"(ESKF::ObsType::ACC_AS_GRAVITY) 与启动自检使用：
    //      · GetLastAccBody()    最近一次去畸变传播用到的 IMU 系比力（已乘量纲因子 acc_scale_factor_、
    //                            未扣加计零偏 ba —— 观测函数内部再扣，保证与 ESKF 的状态定义一致）
    //      · GetLastAngVelBody() 最近一次的去畸变角速度（已扣陀螺零偏 bg）
    //      · GetAccScaleFactor() 量纲因子（静止窗口推断出的 g → m/s² 或原样）
    //      · InitStaticOK()      静止初始化窗口是否被判为"真静止"（false 说明重力对齐不可信，
    //                            日志会打 WARNING；此时靠 ACC_AS_GRAVITY 观测在线纠正）
    // ---------------------------------------------------------------------------------------------
    Vec3d GetLastAccBody() const { return acc_body_last_; }
    Vec3d GetLastAngVelBody() const { return angvel_body_last_; }
    double GetAccScaleFactor() const { return acc_scale_factor_; }
    bool InitStaticOK() const { return init_static_ok_; }
    double GetInitAccNorm() const { return init_acc_norm_; }
    double GetInitAccDev() const { return init_acc_dev_; }
    double GetInitGyrNorm() const { return init_gyr_norm_; }

    /// 个人修改（方案C）：静止窗口判定阈值（yaml 可选键覆盖，缺省值见成员初值）
    void SetInitStaticParams(double acc_tol, double gyr_tol, double std_rel_tol, double min_duration_s,
                             double max_duration_s, int max_wait_count) {
        init_static_acc_tol_ = acc_tol;
        init_static_gyr_tol_ = gyr_tol;
        init_static_std_rel_tol_ = std_rel_tol;
        init_min_duration_s_ = min_duration_s;
        init_max_duration_s_ = max_duration_s;
        init_max_wait_count_ = max_wait_count;
    }

    /// 个人修改（方案C）：过程噪声维度随状态扩到 15（新增加计零偏随机游走）
    ESKF::ProcessNoiseType Q_;
    Vec3d cov_acc_;
    Vec3d cov_gyr_;
    Vec3d cov_acc_scale_;
    Vec3d cov_gyr_scale_;
    Vec3d cov_bias_gyr_;
    Vec3d cov_bias_acc_;

   private:
    void IMUInit(const MeasureGroup &meas, ESKF &kf_state, int &N);
    void UndistortPcl(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &pcl_out);

    /// 个人修改（方案C）：由"静止窗口的平均比力模长"推断 IMU 的量纲因子。
    ///   >0 返回推断出的比例因子（把原始读数换算到 m/s²）；<0 表示无法判断（调用方保持原值并告警）。
    static double InferAccUnitScale(double mean_acc_norm);

    static inline constexpr int max_init_count_ = 20;

    PointCloudType::Ptr cur_pcl_un_ = nullptr;
    lightning::IMUPtr last_imu_ = nullptr;
    std::deque<lightning::IMUPtr> imu_queue_;

    std::vector<Pose6D> imu_pose_;
    Mat3d R_lidar_imu_ = Mat3d ::Identity();
    Vec3d t_lidar_mu_ = Vec3d ::Zero();
    Vec3d mean_acc_ = Vec3d::Zero();
    Vec3d mean_gyr_ = Vec3d::Zero();
    Vec3d angvel_last_ = Vec3d ::Zero();
    Vec3d acc_s_last_ = Vec3d ::Zero();
    double acc_scale_factor_ = 1.0;

    /// 个人修改（方案C）：最近一次的 IMU 系比力/角速度（给重力观测与自检用）
    Vec3d acc_body_last_ = Vec3d::Zero();
    Vec3d angvel_body_last_ = Vec3d::Zero();

    /// 个人修改（方案C）：静止窗口判据与自检量
    bool init_static_ok_ = false;      // 初始化窗口是否判为静止（重力对齐是否可信）
    double init_acc_norm_ = 0.0;       // 静止窗口平均比力模长（换算后，m/s²）
    double init_acc_dev_ = 0.0;        // | |a| - g |（m/s²）
    double init_gyr_norm_ = 0.0;       // 静止窗口平均角速度模长（rad/s）
    double init_time_first_ = 0.0;     // 静止窗口起始时间戳
    double init_static_acc_tol_ = 0.15;       // 允许的 |a|-g 偏差（m/s²，≈0.015 g）
    double init_static_gyr_tol_ = 0.10;       // 允许的平均角速度（rad/s，≈5.7°/s）
    double init_static_std_rel_tol_ = 0.05;   // 允许的比力抖动（相对均值，无单位）
    double init_min_duration_s_ = 0.5;        // 认为"窗口足够长"的最短时长（s）
    double init_max_duration_s_ = 5.0;        // 最长等待时长（s，超时后带 WARNING 强制完成）
    int init_max_wait_count_ = 100;           // 最长等待帧数（约 10 s@10Hz，与上者取先到者）

    /// 个人修改（方案C）：静止幅值误差允许用"比例修正"吸收的相对上限（超过则认为不是标度误差）
    static inline constexpr double kAccMagnitudeScaleFixTol = 0.03;

    double last_lidar_end_time_ = 0;
    int init_iter_num_ = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;

    bool use_imu_filter_ = true;
    IMUFilter filter_;
};

inline ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_.setZero();
    /// 个人修改（方案C）：过程噪声由 12 维扩到 15 维，第 9~11 个对角项 = 加计零偏随机游走
    ///   （前 9 项含义不变：0~2 陀螺白噪声、3~5 加计白噪声、6~8 陀螺零偏随机游走）
    Q_.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    cov_acc_ = Vec3d(0.1, 0.1, 0.1);
    cov_gyr_ = Vec3d(0.1, 0.1, 0.1);
    cov_bias_gyr_ = Vec3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = Vec3d(0.0001, 0.0001, 0.0001);
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    last_imu_.reset(new lightning::IMU());
}

inline ImuProcess::~ImuProcess() {}

inline void ImuProcess::Reset() {
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    angvel_last_.setZero();
    acc_scale_factor_ = 1.0;

    /// 个人修改（方案C）：自检量一并复位
    acc_body_last_.setZero();
    angvel_body_last_.setZero();
    init_static_ok_ = false;
    init_acc_norm_ = 0.0;
    init_acc_dev_ = 0.0;
    init_gyr_norm_ = 0.0;

    imu_need_init_ = true;
    init_iter_num_ = 1;
    imu_queue_.clear();
    imu_pose_.clear();
    last_imu_.reset(new lightning::IMU());
    cur_pcl_un_.reset(new PointCloudType());
}

/// 个人修改（方案C）：量纲推断（原来是写在 Process() 里的一段，抽出来供 IMUInit/Process 共用）
inline double ImuProcess::InferAccUnitScale(double mean_acc_norm) {
    if (mean_acc_norm > 0.5 && mean_acc_norm < 1.5) {
        return G_m_s2;  // 读数以 g 为单位
    } else if (mean_acc_norm > 7.0 && mean_acc_norm < 12.0) {
        return 1.0;  // 读数已经是 m/s²
    }
    return -1.0;  // 无法判断
}

inline void ImuProcess::SetExtrinsic(const Vec3d &transl, const Mat3d &rot) {
    t_lidar_mu_ = transl;
    R_lidar_imu_ = rot;
}

inline void ImuProcess::SetGyrCov(const Vec3d &scaler) { cov_gyr_scale_ = scaler; }

inline void ImuProcess::SetAccCov(const Vec3d &scaler) { cov_acc_scale_ = scaler; }

inline void ImuProcess::SetGyrBiasCov(const Vec3d &b_g) { cov_bias_gyr_ = b_g; }

inline void ImuProcess::SetAccBiasCov(const Vec3d &b_a) { cov_bias_acc_ = b_a; }

inline void ImuProcess::IMUInit(const MeasureGroup &meas, ESKF &kf_state, int &N) {
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_ **/

    Vec3d cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        init_time_first_ = meas.imu_.front()->timestamp;  // 个人修改（方案C）：窗口起点
        const auto &imu_acc = meas.imu_.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu_.front()->angular_velocity;
        mean_acc_ = imu_acc;
        mean_gyr_ = gyr_acc;
    }

    for (const auto &imu : meas.imu_) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc = imu_acc;
        cur_gyr = gyr_acc;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ =
            cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ =
            cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }

    auto init_state = kf_state.GetX();
    init_state.timestamp_ = meas.imu_.back()->timestamp;

    // =============================================================================================
    //  个人修改（2026-10-02，方案C：姿态/重力根治）—— 初始化窗口"静止判据 + 幅值一致性"
    //
    //  【为什么必须要这一步】原初始化窗口只有 max_init_count_(20) 个 IMU 采样（约 0.1~0.2 s），
    //    且不检查车辆是否静止：只要这 0.1~0.2 s 里车在加速/转弯，mean_acc_ 就同时包含重力与
    //    运动加速度，用它对齐出来的"竖直"就被污染成了一个歪掉的世界系。
    //    实测：0.2 s 内 4 m/s² 的加减速即可让对齐结果歪 20° 以上；而世界系一旦歪掉，
    //    LIO 轨迹与地图会整体刚性倾斜（实测本 bag 倾斜 24.32°），表现为 z 轴持续下漂，
    //    且因为没有任何绝对竖直参考（原 ESKF 无加计零偏/重力状态），事后无法纠回。
    //
    //  【本步做三件事】
    //    ① 量纲推断提前到初始化内部（原来在 Process() 里、初始化完成后才算，导致此处用不上）；
    //    ② 静止判据：| |a| - g | 小 且 平均角速度小 且 比力抖动小 ⇒ 才认为对齐可信；
    //       不满足时【不完成初始化】（继续累积窗口），最多等 init_max_wait_count_ 帧后强制完成
    //       并打 WARNING（给出实测值，便于判断是不是"上车即开动"）；
    //    ③ 幅值一致性：静止窗口内 |a| 与 g 的相对偏差在 3% 以内时，用【比例修正】把它吸收进
    //       acc_scale_factor_（本平台为 +0.42%，即 0.041 m/s²）。
    //       这样做的意义：ESKF 的 get_f() 里 a 是 w.r.t. 世界系使用的，若 |a| 有 0.42% 的系统偏差，
    //       预测就会持续带一个 ~0.04 m/s² 的"斜向"加速度（正是 z 漂移的直接来源之一）。
    //       放在比例上而不是 ba 上，是因为它随加速度成比例（1g 处等效），残留部分仍由在线 ba 兜底。
    // =============================================================================================
    const double mean_acc_norm_raw = mean_acc_.norm();
    const double unit_scale = InferAccUnitScale(mean_acc_norm_raw);
    if (unit_scale > 0) {
        acc_scale_factor_ = unit_scale;
    }

    init_acc_norm_ = mean_acc_norm_raw * acc_scale_factor_;
    init_acc_dev_ = std::fabs(init_acc_norm_ - G_m_s2);
    init_gyr_norm_ = mean_gyr_.norm();
    const double acc_std_rel =
        (mean_acc_norm_raw > 1e-6)
            ? std::sqrt(std::max(cov_acc_.cwiseAbs().maxCoeff(), 0.0)) / mean_acc_norm_raw
            : 1.0;
    init_static_ok_ = (init_acc_dev_ < init_static_acc_tol_) && (init_gyr_norm_ < init_static_gyr_tol_) &&
                      (acc_std_rel < init_static_std_rel_tol_);

    double acc_scale_fix = 1.0;
    if (unit_scale > 0 && init_static_ok_ && (init_acc_dev_ / G_m_s2) < kAccMagnitudeScaleFixTol) {
        acc_scale_fix = G_m_s2 / std::max(init_acc_norm_, 1e-6);
        acc_scale_factor_ *= acc_scale_fix;
        init_acc_norm_ = G_m_s2;  // 修正后静止幅值严格等于 g
        init_acc_dev_ = 0.0;
    }

    LOG(INFO) << "[imu init] N=" << N << ", window=" << std::setprecision(3) << (meas.imu_.back()->timestamp -
                                                                                init_time_first_)
              << "s, |a|=" << init_acc_norm_ << " m/s^2 (raw " << mean_acc_norm_raw << "), scale=" << acc_scale_factor_
              << " (fix " << acc_scale_fix << "), |w|=" << init_gyr_norm_ << " rad/s, acc_std_rel=" << acc_std_rel
              << ", static=" << (init_static_ok_ ? "yes" : "NO");

    /// 初始姿态：把静止时的“比力方向”（即竖直向上）显式对齐到世界系 +z，世界系重力固定为 (0,0,-G)。
    ///
    /// 原实现是直接把 IMU 系测得的重力方向写入 grav_：
    ///     init_state.grav_ = -mean_acc_ / mean_acc_.norm() * G_m_s2;
    /// 而 init_state.rot_ 保持默认的单位阵，这等价于令【世界系 = IMU 初始姿态系】。
    /// 当 IMU 水平安装（z 朝上）时恰好得到“世界系 z 竖直向上”的正确结果；
    /// 但 IMU 竖装时（例如重力沿 IMU +x 轴，静止比力 ≈ 9.81 -0.18 0.06），
    /// 世界系 z 轴会变成车体前进方向，表现为：
    ///   · odom/TF 中车体“掉头”看似绕前进方向翻滚（RViz 里机器人往天上翻）；
    ///   · 依赖“z 朝上”的下游模块（g2p5 栅格地图假设地面为 z=常数、后端 with_height 高度约束、
    ///     定位端 mask_z/mask_roll_pitch 自由度屏蔽）全部失效。
    /// 这里改为显式重力对齐：世界系 z 轴严格竖直向上，重力固定为 (0,0,-G)。
    /// 注意：对齐后首帧位姿不再等于单位阵，任何依赖“世界系 = IMU 初始姿态系”的解析式
    ///      外参/锚点计算都会多转一个 q_align。
    /// 个人修改（方案C）补充：只有 init_static_ok_ 时这一步才是"重力对齐"；否则它是"比力对齐"
    ///      （可能包含运动加速度），此时会在上面打 WARNING，并由 laser_mapping 的重力观测在线纠正。
    const Vec3d up_in_imu = mean_acc_.normalized();
    const Quatd q_align = Quatd::FromTwoVectors(up_in_imu, Vec3d(0, 0, 1));
    init_state.rot_ = SO3(q_align.toRotationMatrix());
    init_state.grav_ = Vec3d(0.0, 0.0, -G_m_s2);  // 世界系重力（z 轴竖直向上）

    init_state.bg_ = mean_gyr_;
    init_state.ba_.setZero();  // 个人修改（方案C）：静止幅值误差已由 acc_scale_factor_ 吸收，ba 从 0 起在线估计
    kf_state.ChangeX(init_state);

    /// 个人修改（方案C）：收紧初始协方差。
    ///   原实现 init_P.setIdentity() ⇒ 姿态块方差 1.0 rad²（≈57°²）、位置 1 m²、速度 1 (m/s)²，
    ///   即"初始姿态几乎完全不可信、位置/速度也随便动"。后果：
    ///     · 首帧 LiDAR 更新会把姿态按第一帧点云的（可能退化的）几何拉过去，
    ///       之后 P 迅速收缩 ⇒ 姿态被"冻住"，重力方向无人校正（这正是 24° 倾斜被长期保持的机制）；
    ///     · 位置块过松会让首帧产生米级的位置跳变，后续靠 LiDAR 慢慢拉回。
    ///   现在按"重力对齐后的真实不确定度"给出：姿态 1°、位置 10 cm、速度 0.2 m/s、
    ///   陀螺零偏 3e-3 rad/s、加计零偏 0.05 m/s²。
    auto init_P = kf_state.GetP();
    init_P.setZero();
    constexpr double kInitPosStd = 0.1;      // m
    constexpr double kInitRotStd = 1.0 * M_PI / 180.0;  // rad（≈1°，静止重力对齐的噪声量级）
    constexpr double kInitVelStd = 0.2;      // m/s
    constexpr double kInitBgStd = 3e-3;      // rad/s
    constexpr double kInitBaStd = 0.05;      // m/s²
    init_P.block<NavState::kBlockDim, NavState::kBlockDim>(NavState::kPosIdx, NavState::kPosIdx) =
        kInitPosStd * kInitPosStd * Mat3d::Identity();
    init_P.block<NavState::kBlockDim, NavState::kBlockDim>(NavState::kRotIdx, NavState::kRotIdx) =
        kInitRotStd * kInitRotStd * Mat3d::Identity();
    init_P.block<NavState::kBlockDim, NavState::kBlockDim>(NavState::kVelIdx, NavState::kVelIdx) =
        kInitVelStd * kInitVelStd * Mat3d::Identity();
    init_P.block<NavState::kBlockDim, NavState::kBlockDim>(NavState::kBgIdx, NavState::kBgIdx) =
        kInitBgStd * kInitBgStd * Mat3d::Identity();
    init_P.block<NavState::kBlockDim, NavState::kBlockDim>(NavState::kBaIdx, NavState::kBaIdx) =
        kInitBaStd * kInitBaStd * Mat3d::Identity();
    kf_state.ChangeP(init_P);

    // LOG(INFO) << "P diag: " << init_P.diagonal().transpose();

    last_imu_ = meas.imu_.back();
}

inline void ImuProcess::UndistortPcl(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &pcl_out) {
    /*** add the imu_ of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);
    const double &imu_end_time = v_imu.back()->timestamp;

    const double &pcl_beg_time = meas.lidar_begin_time_;
    const double &pcl_end_time = meas.lidar_end_time_;

    /*** Initialize IMU pose ***/
    auto imu_state = kf_state.GetX();
    imu_pose_.clear();
    imu_pose_.emplace_back(0.0, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix());

    /*** forward propagation at each imu_ point ***/
    Vec3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    Mat3d R_imu;

    double dt = 0;
    Vec3d acc = Vec3d::Zero();
    Vec3d gyro = Vec3d::Zero();

    if (use_imu_filter_) {
        for (auto &imu : v_imu) {
            auto imu_f = filter_.Filter(*imu);
            *imu = imu_f;
        }
    }

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (tail->timestamp < last_lidar_end_time_) {
            continue;
        }

        angvel_avr = .5 * (head->angular_velocity + tail->angular_velocity);
        acc_avr = .5 * (head->linear_acceleration + tail->linear_acceleration);
        acc_avr = acc_avr * acc_scale_factor_;

        if (head->timestamp < last_lidar_end_time_) {
            dt = tail->timestamp - last_lidar_end_time_;
        } else {
            dt = tail->timestamp - head->timestamp;
        }

        acc = acc_avr;
        gyro = angvel_avr;

        if (dt > 0.1) {
            LOG(ERROR) << "get abnormal dt: " << dt;
            kf_state.SetTime((*it_imu)->timestamp);
            break;
        }

        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        kf_state.Predict(dt, Q_, gyro, acc);

        // LOG(INFO) << "gyro: " << gyro.transpose() << ", dt: " << dt;

        // LOG(INFO) << "acc: " << acc.transpose() << " grav: " << kf_state.GetX().grav_.norm()
        //           << ", vel: " << kf_state.GetX().vel_.transpose() << ", dt: " << dt;

        /* save the poses at each IMU measurements */
        imu_state = kf_state.GetX();
        angvel_last_ = angvel_avr - imu_state.bg_;
        /// 个人修改（方案C）：去畸变用的比力同样要扣掉加计零偏 ba。
        ///   原实现 acc_s_last_ = rot_ * acc_avr（不扣任何加计零偏），与 get_f() 里的
        ///   a_inertial = rot_*(acce - ba_) 不一致 ⇒ 去畸变与预测"差一个 0.041 m/s² 的常量"，
        ///   在帧内 0.1 s 上体现为几毫米的形变偏差，累积到地图上就是系统性 z 偏移。
        acc_s_last_ = imu_state.rot_ * (acc_avr - imu_state.ba_);
        angvel_body_last_ = angvel_last_;          // 个人修改（方案C）：供重力观测的静止门限判断
        acc_body_last_ = acc_avr;                  // 个人修改（方案C）：IMU 系比力（已乘量纲、未扣 ba）
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav_[i];
        }

        double &&offs_t = tail->timestamp - pcl_beg_time;
        imu_pose_.emplace_back(
            Pose6D(offs_t, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.Predict(dt, Q_, gyro, acc);

    imu_state = kf_state.GetX();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** sort point clouds by offset time ***/
    pcl_out = meas.scan_;
    std::sort(pcl_out->points.begin(), pcl_out->points.end(),
              [](const PointType &p1, const PointType &p2) { return p1.time < p2.time; });

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out->empty()) {
        return;
    }

    auto it_pcl = pcl_out->points.end() - 1;
    for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = (head->rot);
        vel_imu = (head->vel);
        pos_imu = (head->pos);
        acc_imu = (tail->acc);
        angvel_avr = (tail->gyr);

        for (; it_pcl->time / double(1000) > head->offset_time && it_pcl != pcl_out->points.begin(); it_pcl--) {
            dt = it_pcl->time / double(1000) - head->offset_time;

            /// dt 有时候存在非法数据
            if (dt < 0 || dt > lo::lidar_time_interval) {
                // LOG(WARNING) << "find abnormal dt in cloud: " << dt;
                continue;
            }

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * p_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            Mat3d R_i(R_imu * math::exp(angvel_avr, dt).matrix());

            Vec3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            Vec3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos_);
            Vec3d p_compensate = R_lidar_imu_.transpose() *
                                 (imu_state.rot_.inverse() * (R_i * (R_lidar_imu_ * P_i + t_lidar_mu_) + T_ei) -
                                  t_lidar_mu_);  // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            // if (it_pcl == pcl_out->points.begin()) {
            //     break;
            // }
        }
    }
}

inline void ImuProcess::Process(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &scan) {
    if (meas.imu_.empty()) {
        return;
    }

    if (imu_need_init_) {
        /// The very first lidar frame
        IMUInit(meas, kf_state, init_iter_num_);

        imu_need_init_ = true;

        last_imu_ = meas.imu_.back();

        auto imu_state = kf_state.GetX();

        /// 个人修改（方案C）：初始化完成条件由"采样数够了"改为
        ///   ① 窗口够长（>= init_min_duration_s_） **且** 判为静止（init_static_ok_）；
        ///   ② 或者等到超时（init_max_duration_s_ / init_max_wait_count_ 先到者）→ 带 WARNING 强制完成。
        ///   原实现只要 max_init_count_(20) 个采样就完成（≈0.1~0.2 s），车辆只要在这 0.1~0.2 s 内
        ///   有加减速/转弯，"重力对齐"就会被运动加速度污染（实测 4 m/s² 可造成 >20° 倾斜），
        ///   而世界系一旦歪掉，整张地图与轨迹都会刚性倾斜 ⇒ z 轴持续下漂。
        const double init_window = meas.imu_.back()->timestamp - init_time_first_;
        const bool init_ready_by_count = (init_iter_num_ > max_init_count_) && (init_window >= init_min_duration_s_);
        const bool init_timeout = (init_window >= init_max_duration_s_) || (init_iter_num_ > init_max_wait_count_);

        if ((init_ready_by_count && init_static_ok_) || init_timeout) {
            imu_need_init_ = false;

            if (!init_static_ok_) {
                LOG(ERROR) << "[方案C] IMU 初始化完成时窗口【未判为静止】，重力对齐不可信：|a|=" << init_acc_norm_
                           << " m/s^2 (偏差 " << init_acc_dev_ << "), |w|=" << init_gyr_norm_ << " rad/s, 采样 "
                           << init_iter_num_ << ", 窗口 " << init_window << " s。"
                           << "世界系可能整体倾斜 ⇒ 请让车【静止】后重新建图；"
                           << "运行期将由 ACC_AS_GRAVITY 观测与地图重力再对齐兜底。";
            } else if (init_timeout) {
                LOG(WARNING) << "[方案C] IMU 初始化等待超时（窗口 " << init_window << " s, 采样 " << init_iter_num_
                             << "），按当前窗口完成初始化（static=yes）";
            }

            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;

            LOG(INFO) << "imu init done, bg: " << imu_state.bg_.transpose() << ", ba: " << imu_state.ba_.transpose()
                      << ", grav: " << imu_state.grav_.transpose() << ", acc scale: " << acc_scale_factor_
                      << ", mean: " << mean_acc_.transpose() << ", " << mean_gyr_.transpose() << ", |a|="
                      << init_acc_norm_ << " m/s^2, static: " << (init_static_ok_ ? "yes" : "NO") << ", window "
                      << init_window << "s";

            LOG(INFO) << "[方案C] 初始姿态(世界系 z = 静止比力方向): T_w_imu 首帧 rpy(deg) = ["
                      << (imu_state.rot_.matrix().eulerAngles(0, 1, 2).transpose() * 180.0 / M_PI)
                      << "] （这是世界系与重力的关系，不是车体姿态；车体姿态另见 LIO state(vehicle) 日志）";
        } else {
            LOG(INFO) << "waiting for imu init ... N=" << init_iter_num_ << ", window=" << std::setprecision(3)
                      << init_window << "s (需 >=" << init_min_duration_s_ << "s 且静止), |a|=" << init_acc_norm_
                      << " dev=" << init_acc_dev_ << ", |w|=" << init_gyr_norm_
                      << ", static=" << (init_static_ok_ ? "yes" : "NO");
        }

        return;
    }

    Timer::Evaluate([&, this]() { UndistortPcl(meas, kf_state, scan); }, "Undistort Pcl");
}
}  // namespace lightning

#endif
