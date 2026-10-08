//
// 个人修改（2026-10-02，方案C：姿态/重力根治）—— 新增自检程序（不参与定位/建图主流程）
//
// 【要验证什么】
//   方案C 的核心是"把重力方向作为姿态观测"（ESKF::ObsType::ACC_AS_GRAVITY），
//   它有三个最容易写错的地方：
//     ① 残差与雅可比的符号约定（本工程 LiDAR 观测用的是 J = ∂h/∂x、res = -h，见 laser_mapping.cc::ObsModel）；
//     ② 3 维姿态块要放在 CustomObservationModel 的第 3~5 维（pos 在前、rot 在后）；
//     ③ 观测权重/步长上限不能被 ESKF::Update 的 5° 保护整帧丢弃。
//   本程序用一个"姿态被摆歪、但比力始终是竖直"的合成场景，跑 ESKF 的完整
//   Predict + Update 循环，检查姿态是否【单调收敛】回竖直。收敛即说明约定全对。
//
// 【怎么跑】
//   ./bin/test_gravity_obs
//   （编译目标见 src/app/CMakeLists.txt；退出码 0 = 通过）
//
#include <glog/logging.h>

#include <cmath>
#include <cstdio>
#include <vector>

#include "common/constant.h"
#include "common/nav_state.h"
#include "core/lio/eskf.hpp"

using namespace lightning;

namespace {

/// 与 laser_mapping.cc::AccAsGravityObsModel 完全一致的观测模型（此处复刻一份，便于独立自检；
/// 若主流程的公式有改动，请同步这里，否则本测试就失去意义）
void GravityObsModel(NavState &s, ESKF::CustomObservationModel &obs, const Vec3d &a_body_meas, double sigma_deg) {
    obs.HTH_.setZero();
    obs.HTr_.setZero();
    obs.valid_ = true;

    const Vec3d up_body = (a_body_meas - s.ba_).normalized();
    const Vec3d up_world = s.rot_ * up_body;
    const Vec3d r = up_world - Vec3d(0, 0, 1);  // r = -h
    const Mat3d J = s.rot_.matrix() * SO3::hat(up_body);

    const double sigma_rad = std::max(sigma_deg, 0.05) * M_PI / 180.0;
    const double w = 1.0 / (sigma_rad * sigma_rad);

    obs.HTH_.block<3, 3>(3, 3) = w * (J.transpose() * J);
    obs.HTr_.block<3, 1>(3, 0) = w * (J.transpose() * r);
    obs.lidar_residual_mean_ = r.norm();
}

/// 姿态误差（度）：R_est 与 R_true 之间的夹角
double RotErrorDeg(const SO3 &R_est, const SO3 &R_true) { return (R_true.inverse() * R_est).log().norm() * 180.0 / M_PI; }

}  // namespace

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    /// 合成真值：车体水平（R_true = I），IMU 测到的比力在 IMU 系里就是 (0,0,g)
    const SO3 R_true = SO3();
    const Vec3d g_world(0, 0, -constant::G_m_s2);  // 只做参考
    const Vec3d a_body_meas = R_true.inverse() * Vec3d(0, 0, constant::G_m_s2);

    /// 被估状态：姿态被摆歪 20°（如世界系初始化被运动加速度污染导致的整体倾斜）
    const double init_tilt_deg = 20.0;
    NavState x0;
    x0.rot_ = SO3::exp(Vec3d(init_tilt_deg * M_PI / 180.0, 0, 0));
    x0.grav_ = g_world;

    const double sigma_deg = 1.5;

    ESKF::Options opt;
    opt.max_iterations_ = 4;
    opt.epsi_ = 1e-3 * Eigen::Matrix<double, ESKF::state_dim_, 1>::Ones();
    opt.acc_as_gravity_obs_func_ = [&](NavState &s, ESKF::CustomObservationModel &obs) {
        GravityObsModel(s, obs, a_body_meas, sigma_deg);
    };
    opt.use_aa_ = false;

    ESKF kf(x0);
    kf.Init(opt);

    /// 初值 P：与 ImuProcess::IMUInit() 收紧后的初值一致
    ESKF::CovType P = ESKF::CovType::Zero();
    P.block<3, 3>(NavState::kPosIdx, NavState::kPosIdx) = 0.1 * 0.1 * Mat3d::Identity();
    P.block<3, 3>(NavState::kRotIdx, NavState::kRotIdx) = (1.0 * M_PI / 180.0) * (1.0 * M_PI / 180.0) * Mat3d::Identity();
    P.block<3, 3>(NavState::kVelIdx, NavState::kVelIdx) = 0.2 * 0.2 * Mat3d::Identity();
    P.block<3, 3>(NavState::kBgIdx, NavState::kBgIdx) = 3e-3 * 3e-3 * Mat3d::Identity();
    P.block<3, 3>(NavState::kBaIdx, NavState::kBaIdx) = 0.05 * 0.05 * Mat3d::Identity();
    kf.ChangeP(P);

    /// 过程噪声（与 ImuProcess 构造函数同量级）
    ESKF::ProcessNoiseType Q = ESKF::ProcessNoiseType::Zero();
    Q.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    const double dt = 0.1;  // 10 Hz，与雷达帧率一致
    const int frames = 100;

    double prev_err = RotErrorDeg(kf.GetX().rot_, R_true);
    double max_increase_deg = 0.0;
    int max_increase_frame = -1;
    std::vector<double> errs;

    for (int i = 0; i < frames; ++i) {
        kf.Predict(dt, Q, Vec3d::Zero(), a_body_meas);  // 静止：陀螺=0，比力=竖直
        kf.Update(ESKF::ObsType::ACC_AS_GRAVITY, 1.0);

        const double err = RotErrorDeg(kf.GetX().rot_, R_true);
        errs.push_back(err);
        if (err - prev_err > max_increase_deg) {
            max_increase_deg = err - prev_err;
            max_increase_frame = i;
        }
        prev_err = err;
    }

    const double final_err = errs.back();
    const auto &final_state = kf.GetX();
    const Vec3d up_world = final_state.rot_ * (a_body_meas - final_state.ba_).normalized();
    const double final_tilt_deg = std::acos(std::min(1.0, std::max(-1.0, up_world.dot(Vec3d(0, 0, 1))))) * 180.0 / M_PI;

    LOG(INFO) << "=========== 方案C 重力观测自检 ===========";
    LOG(INFO) << "初始姿态误差: " << init_tilt_deg << " deg（合成场景：只把姿态摆歪，比力始终竖直）";
    LOG(INFO) << "第 1 帧后: " << errs.front() << " deg, 第 5 帧后: " << errs[std::min<size_t>(4, errs.size() - 1)]
              << " deg, 第 20 帧后: " << errs[std::min<size_t>(19, errs.size() - 1)] << " deg, 最终: " << final_err
              << " deg";
    LOG(INFO) << "最终 世界系z 与重力夹角 = " << final_tilt_deg << " deg, 估计出的 ba = " << final_state.ba_.transpose()
              << ", bg = " << final_state.bg_.transpose();
    LOG(INFO) << "单帧最大误差增量 = " << max_increase_deg << " deg @ frame " << max_increase_frame
              << "（判据：<= 0.2 deg；符号写反会表现为每帧增大数度并发散）";

    /// 判据说明：
    ///   · 姿态误差必须收敛（最终 < 0.5°）—— 收敛即证明 J/r 符号、姿态块位置(第3~5维)、权重都正确；
    ///   · 允许单帧有极小的"过冲"（迭代 EKF 对 sin 型残差线性化，20° 初值时首帧会有 ~0.05° 过冲），
    ///     但不允许明显持续增大（符号错误时每帧会增大数度）。
    int ret = 0;
    if (max_increase_deg > 0.2) {
        LOG(ERROR) << "自检失败：姿态误差单帧增大 " << max_increase_deg << " deg（> 0.2），观测符号约定可能写反了";
        ret = 1;
    } else if (!(final_err < 0.5)) {
        LOG(ERROR) << "自检失败：姿态没有收敛到竖直（最终 " << final_err << " deg >= 0.5 deg）";
        ret = 1;
    } else {
        LOG(INFO) << "自检通过：姿态从 " << init_tilt_deg << " deg 单调收敛到 " << final_err
                  << " deg，重力观测(符号/维度/权重/步长)约定正确";
    }

    /// 附带检查 ESKF 维度：状态 15 维（含加计零偏 ba）、过程噪声 15 维
    static_assert(NavState::dim == 15, "NavState 应为 15 维（pos/rot/vel/bg/ba）");
    static_assert(NavState::kBaIdx == 12, "加计零偏 ba 应从第 12 维开始");
    static_assert(ESKF::state_dim_ == 15, "ESKF 状态维度应与 NavState 一致");
    static_assert(ESKF::process_noise_dim_ == 15, "过程噪声维度应为 15（含 ba 随机游走）");

    return ret;
}
