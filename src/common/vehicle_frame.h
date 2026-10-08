//
// 个人修改：IMU 姿态 → 车辆(base_link)姿态 的坐标系换算工具（新增文件）
//
// 【为什么需要这个文件】
//   本平台（Airy 96）的 IMU 是【竖装】的：标称安装 x = 向下(指向地心), y = 左侧, z = 前进方向；
//   而车辆(base_link)系习惯为 x = 前进, y = 左, z = 上。LIO/ESKF 内部完全按 IMU 系工作
//   （世界系正确、不要动它），但把该姿态直接当成"车体姿态"输出到 UI / 日志 / TF 时会出现：
//     1) 画出来的车 x 轴指地、z 轴指前 ⇒ 看上去"车绕 y 轴立起来 90°、要上天"；
//     2) 用"绕 x/y/z 各多少度"读姿态时 pitch 恒为 90°、真正拐弯跑到 roll 上
//        （SO3::angleX/Y/Z 是 2x2 子块的 SVD log，不是真正的 RPY），±180° 附近还会乱跳。
//
// 【方案A 的做法】不动 ESKF、不动地图、不动世界系，只在"输出/显示边界"把姿态右乘 Mᵀ：
//        T_w_vehicle = T_w_body * SE3(Mᵀ, 0)     // 位置向量与坐标系无关，故只换旋转
//   其中 M = R_vehicle_imu = "IMU 三轴在车辆系下的表示"（第 i 列 = IMU 第 i 轴在车辆系的方向）。
//   标称竖装（x=下, y=左, z=前）对应 M = [0 0 1; 0 1 0; -1 0 0] = R_y(+90°)。
//
// 【方案A+（本版本）：M 不写死，改由 yaml 的 common 段配置，三种方式】
//   1) common.vehicle_imu_mount: auto      ← 推荐：用 IMU 静止初始化的重力方向自动定"上"，
//        再用 common.vehicle_imu_forward_axis（默认 z，可写 -z 等）定车头方向；
//        对安装【任意倾斜】自动适应，不需要人给倾角。
//   2) common.vehicle_imu_mount: identity | x_down | x_up   固定安装挡（Airy 标称竖装 = x_down）
//   3) common.vehicle_imu_R: [9 个数，行主序]               直接给 M，可描述任意安装（优先级最高）
//   三项都不写 = identity = 不做换算（与"加入方案A之前"行为一致），laser_mapping.cc 会给出 WARNING。
//   ★本平台实测：标称"IMU x=下"与实车重力方向相差约 11.25°（见 doc/airy96_config_guide.md 第 11 节），
//     只写标称值会让车始终带着约 11° 的歪，故推荐 auto。
//
// 【与雷达安装无关】雷达（水平/倾斜/不同批次的 Airy）与 IMU 的相对安装由 yaml 的
//   fasterlio.extrinsic_R / extrinsic_T（"雷达 → IMU"）描述，本文件不涉及、也无需修改；
//   换雷达安装姿态只需改这两个外参（另需同步 roi.height_*、g2p5.floor_height 等按雷达系 z
//   定义的高度参数），本文件与方案A 的代码都不用动。
//
#pragma once

#include <cctype>
#include <cmath>
#include <string>

#include "common/eigen_types.h"

namespace lightning {
namespace vehicle_frame {

/// 个人修改（方案A+）：安装旋转 M = R_vehicle_imu 的运行期存放处（缺省单位阵 = 不做换算）
inline Mat3d &VehicleImuRotationRef() {
    static Mat3d M = Mat3d::Identity();
    return M;
}

/// 个人修改：IMU 系 → 车辆(base_link)系 的旋转 M = R_vehicle_imu（由 yaml 配置或重力自动对齐得到）
inline const Mat3d &R_vehicle_imu() { return VehicleImuRotationRef(); }

/// 个人修改（方案A+）：设置安装旋转并做合法性校验（必须正交、det = +1），非法返回 false
inline bool SetVehicleImuRotation(const Mat3d &R) {
    if (R.allFinite() == false) {
        return false;
    }
    if ((R.transpose() * R - Mat3d::Identity()).cwiseAbs().maxCoeff() > 1e-6) {
        return false;
    }
    if (std::fabs(R.determinant() - 1.0) > 1e-6) {
        return false;
    }
    VehicleImuRotationRef() = R;
    return true;
}

/// 个人修改（方案A+）：车辆(base_link)系 → IMU 系 的旋转 Mᵀ = R_imu_vehicle
inline Mat3d R_imu_vehicle() { return R_vehicle_imu().transpose(); }

/// 个人修改（方案A+）：轴名字 → IMU 系单位向量（"x"/"y"/"z"，可带负号，忽略大小写与空白）
inline bool ImuAxisFromName(const std::string &name, Vec3d *out) {
    std::string s;
    for (char c : name) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            continue;
        }
        s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    double sign = 1.0;
    if (!s.empty() && (s[0] == '-' || s[0] == '+')) {
        sign = (s[0] == '-') ? -1.0 : 1.0;
        s = s.substr(1);
    }
    if (s == "x") {
        *out = Vec3d(sign, 0, 0);
    } else if (s == "y") {
        *out = Vec3d(0, sign, 0);
    } else if (s == "z") {
        *out = Vec3d(0, 0, sign);
    } else {
        return false;
    }
    return true;
}

/**
 * 个人修改（方案A+）：用"静止时的比力方向"自动求安装旋转 —— 对任意倾斜安装都成立。
 *
 *   车辆 +z = 重力反方向（静止时加速度计测到的就是"上"）；
 *   车辆 +x = 指定 IMU 轴在水平面内的投影（车头方向）；
 *   车辆 +y = z × x（右手系，指向车体左侧）。
 *
 * @param up_in_imu     静止时的平均加速度（IMU 系，单位任意，内部归一化）；与
 *                      imu_processing.hpp::IMUInit() 内部使用的 mean_acc_ 是同一个量
 * @param forward_axis  车头方向对应 IMU 的哪个轴（"x"/"y"/"z"，可带负号）
 * @param out           结果 M = R_vehicle_imu
 * @param err           失败原因（可为 nullptr）
 * @return 失败：up 为零；轴名非法；所选轴几乎与重力平行（车头方向不可确定，需换轴或直接用
 *         common.vehicle_imu_R 给出完整矩阵）
 */
inline bool VehicleImuRotationFromGravity(const Vec3d &up_in_imu, const std::string &forward_axis, Mat3d *out,
                                          std::string *err = nullptr) {
    const double n = up_in_imu.norm();
    if (n < 1e-6) {
        if (err) {
            *err = "静止平均加速度为零，无法确定上方向";
        }
        return false;
    }
    const Vec3d up = up_in_imu / n;  // 车辆 +z（在 IMU 系下的表示）

    Vec3d f = Vec3d::Zero();
    if (!ImuAxisFromName(forward_axis, &f)) {
        if (err) {
            *err = "vehicle_imu_forward_axis 非法（应为 x/y/z，可带负号）: " + forward_axis;
        }
        return false;
    }

    const double c = f.dot(up);
    if (std::fabs(c) > 0.98) {  // 与重力夹角小于约 11.5° 时取水平投影不可靠
        if (err) {
            *err = "指定轴与重力方向几乎平行（无法确定车头方向）：请改 vehicle_imu_forward_axis，"
                   "或直接用 common.vehicle_imu_R 给出完整安装矩阵";
        }
        return false;
    }

    const Vec3d x_veh = (f - c * up).normalized();  // 车辆 +x（水平面内）
    const Vec3d y_veh = up.cross(x_veh);            // 车辆 +y = 左（右手系 x=前, y=左, z=上）

    Mat3d M;
    M.row(0) = x_veh.transpose();
    M.row(1) = y_veh.transpose();
    M.row(2) = up.transpose();
    if (!SetVehicleImuRotation(M)) {
        if (err) {
            *err = "由重力推出的矩阵不是合法旋转矩阵";
        }
        return false;
    }
    *out = M;
    return true;
}

/// 个人修改（方案A+）：名称 → 安装旋转（identity/none/off = 不换算；x_down = Airy 标称竖装；x_up = 竖装倒置）
inline bool VehicleImuRotationFromName(const std::string &name, Mat3d *out) {
    std::string s;
    for (char c : name) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            continue;
        }
        s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    if (s.empty() || s == "identity" || s == "none" || s == "off" || s == "no") {
        *out = Mat3d::Identity();
        return true;
    }
    if (s == "x_down") {  // IMU 标称竖装：x=下, y=左, z=前 ⇒ R_y(+90°)
        Mat3d m;
        m << 0, 0, 1,  //
            0, 1, 0,   //
            -1, 0, 0;
        *out = m;
        return true;
    }
    if (s == "x_up") {  // IMU 竖装倒置：x=上, y=左, z=后 ⇒ R_y(-90°)
        Mat3d m;
        m << 0, 0, -1,  //
            0, 1, 0,    //
            1, 0, 0;
        *out = m;
        return true;
    }
    return false;
}

/// 个人修改：T_w_body(IMU 系) → T_w_vehicle(base_link 系)。位置向量与坐标系无关，故只换旋转
inline SE3 ToVehiclePose(const SE3& T_w_body) { return T_w_body * SE3(R_imu_vehicle(), Vec3d::Zero()); }

/// 个人修改：T_w_vehicle(base_link 系) → T_w_body(IMU 系)（需要还原时使用）
inline SE3 ToBodyPose(const SE3& T_w_vehicle) { return T_w_vehicle * SE3(R_vehicle_imu(), Vec3d::Zero()); }

/**
 * 个人修改：把 yaml 里的"雷达 → IMU"外参打包成 SE3（即 T_imu_lidar 与 T_lidar_imu 互逆对）：
 *   p_imu = R * p_lidar + t        // R = extrinsic_R, t = extrinsic_T
 * 命名说明：这里与 imu_processing.hpp::R_lidar_imu_ 是同一个量（都来自 extrinsic_R），
 *   含义是"把雷达系的点变到 IMU 系"；切勿与上面的 M = R_vehicle_imu（IMU→车辆）混淆。
 */
inline SE3 T_imu_lidar(const Mat3d& R_imu_lidar, const Vec3d& t_imu_lidar) { return SE3(R_imu_lidar, t_imu_lidar); }

/**
 * 个人修改：T_w_body(IMU 系) → T_w_lidar(雷达系)。
 * 雷达系点云（关键帧点云、去畸变后的 scan）必须用它才能正确落到地图上，
 * 之前 UI 用 IMU 姿态直接画雷达系点云，会整体差一个 90° 外参。
 */
inline SE3 ToLidarPose(const SE3& T_w_body, const SE3& T_imu_lidar) { return T_w_body * T_imu_lidar; }

/// 个人修改：T_w_lidar(雷达系) → T_w_body(IMU 系)
inline SE3 ToBodyPoseFromLidar(const SE3& T_w_lidar, const SE3& T_imu_lidar) {
    return T_w_lidar * T_imu_lidar.inverse();
}

/**
 * 个人修改：T_w_lidar(雷达系) → T_w_vehicle(base_link 系)
 * 定位模块（lidar_loc/PGO）输出的位姿是"雷达系在地图下的位姿"，对外发布 base_link 前要走这一步。
 */
inline SE3 ToVehiclePoseFromLidar(const SE3& T_w_lidar, const SE3& T_imu_lidar) {
    return ToVehiclePose(ToBodyPoseFromLidar(T_w_lidar, T_imu_lidar));
}

}  // namespace vehicle_frame
}  // namespace lightning
