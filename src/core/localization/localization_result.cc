//
// Created by xiang on 24-4-11.
//

#include "localization_result.h"
#include "common/vehicle_frame.h"
#include "core/lightning_math.hpp"

namespace lightning::loc {

geometry_msgs::msg::TransformStamped LocalizationResult::ToGeoMsg() const {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = math::FromSec(timestamp_);
    msg.child_frame_id = "base_link";

    // 个人修改：pose_ 是【雷达系】在地图下的位姿 T_map_lidar，而此处 child_frame_id 明确是 base_link，
    //   必须换算到车辆(base_link)系：T_map_vehicle = T_map_lidar * T_lidar_imu * Mᵀ。
    //   否则 IMU/雷达竖装的平台（Airy 96）发出去的 base_link 会相差约 90°，下游/RViz 里的车会"绕 y 轴立起来"。
    //   注：T_imu_lidar_ 为单位阵时，本函数行为与修改前一致。
    const SE3 pose_vehicle = vehicle_frame::ToVehiclePoseFromLidar(pose_, T_imu_lidar_);

    msg.transform.translation.x = pose_vehicle.translation().x();
    msg.transform.translation.y = pose_vehicle.translation().y();
    msg.transform.translation.z = pose_vehicle.translation().z();

    msg.transform.rotation.x = pose_vehicle.so3().unit_quaternion().x();
    msg.transform.rotation.y = pose_vehicle.so3().unit_quaternion().y();
    msg.transform.rotation.z = pose_vehicle.so3().unit_quaternion().z();
    msg.transform.rotation.w = pose_vehicle.so3().unit_quaternion().w();

    return msg;
}

NavState LocalizationResult::ToNavState() const {
    NavState ret;
    ret.timestamp_ = timestamp_;
    ret.confidence_ = confidence_;

    // 个人修改：送给 UI 的姿态同样统一用车辆(base_link)系，避免车"绕 y 轴立起来"、
    //   以及用 RPY 读姿态时出现 pitch≈90° 的假象与 ±180° 附近的乱跳。
    const SE3 pose_vehicle = vehicle_frame::ToVehiclePoseFromLidar(pose_, T_imu_lidar_);
    ret.pos_ = pose_vehicle.translation();
    ret.rot_ = pose_vehicle.so3();
    ret.pose_is_ok_ = status_ == LocalizationStatus::GOOD;

    // 个人修改：世界系速度与参考坐标系的选择无关，这里仍用"雷达系姿态 × 体坐标系速度"，
    //   保证速度数值与修改前完全一致（UI 内部再投影到车辆系得到 base_link 系速度）
    ret.vel_ = (pose_.so3() * vel_b_);

    return ret;
}

}  // namespace lightning::loc
