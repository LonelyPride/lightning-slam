//
// Created by xiang on 25-6-23.
//

#include "core/g2p5/g2p5.h"
#include "common/constant.h"
// 个人修改（2026-10-02，第 4 次修复③ 配套）：用 math::OrthonormalizeRotation() 处理 yaml 外参
#include "core/lightning_math.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <map>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

#include "utils/timer.h"
#include "yaml-cpp/yaml.h"

namespace lightning::g2p5 {

G2P5::~G2P5() { Quit(); }

void G2P5::Quit() {
    quit_flag_ = true;

    if (options_.online_mode_) {
        draw_frontend_map_thread_.Quit();
    }

    if (draw_backend_map_thread_.joinable()) {
        draw_backend_map_thread_.join();
    }
}

void G2P5::PushKeyframe(Keyframe::Ptr kf) {
    UL lock(kf_mutex_);
    all_keyframes_.emplace_back(kf);

    if (options_.online_mode_) {
        draw_frontend_map_thread_.AddMessage(kf);
    } else {
        RenderFront(kf);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// 个人修改：g2p5 的工作坐标系 = 【车辆(base_link)系】(x=前, y=左, z=上)
///
/// 【为什么必须换算】（与 GetGlobalMap 漏外参是同一类缺陷，但这里更隐蔽）
///   g2p5 的 3D→2D 投影隐含了三个假设：z 轴向上、地面是 z=const 平面、极角在水平面 (x,y) 内。
///   而本工程：
///     · 关键帧点云 kf->GetCloud() 是【雷达系】(Airy 96 竖装：x=左, y=上, z=前)；
///     · 关键帧位姿 kf->GetOptPose() 是【IMU 系】(x=下, y=左, z=前)；
///     · 雷达系↔IMU 系 的外参 yaml extrinsic_R/T 在本平台是绕"前向轴"的 90° 旋转。
///   于是：
///     1) 位姿直接乘点云（= 补丁前 GetGlobalMap 的写法）差一个 90° 外参；
///     2) 只补 T_imu_lidar 得到的是【雷达系】世界位姿，虽然点云与位姿自洽，但 z 轴仍是"前"
///        而不是"上"，"地面 z=常数""极角在水平面内"依然不成立 —— 3D→2D 会把极角算在
///        "左-上"这个竖直面里，栅格图（map.pgm）整体错乱、地面被切成斜层。
///   因此 g2p5 统一换算到车辆系后再投影：
///       点云：p_veh = R_vehicle_imu * (R_imu_lidar * p_lidar + t_imu_lidar)
///       位姿：T_w_veh = T_w_imu * SE3(R_vehicle_imuᵀ, 0)   (即 vehicle_frame::ToVehiclePose)
///   注意这里走的是【两段各自独立】的外参、各有各的配置来源，既不可互相代偿，也不可假设
///   "IMU 系 = 车体系"：
///       第 1 段 雷达系 → IMU 系：fasterlio.extrinsic_R/T（同一台雷达的【单元内部】固定关系，
///                                换整车安装姿态时【不变】）
///       第 2 段 IMU 系 → 车辆系：common.vehicle_imu_*（IMU 在车上的实际安装姿态；auto 模式
///                                运行时按重力自动求解，可描述任意倾斜）
///   其中 R_vehicle_imu 是运行期才确定/可能变化的量（本平台 auto 解出的结果与标称"x=下"
///   相差约 11.25°，见 laser_mapping.cc:235 的日志），故每次用时现取、不做缓存。
///
/// 【配置语义回归标准含义】本次换算只换【朝向】、不搬原点：换算后点云的原点 = IMU 原点，
///   地面位于该原点下方 h 处，因此 yaml 建议改为：
///       g2p5.lidar_height = h      （原点到地面高度；本平台 IMU 在雷达内、extrinsic_T≈4mm
///                                    ⇒ 与"雷达离地高度"等同，IMU 与雷达分离时须按 IMU 填）
///       g2p5.floor_height = -h     （地面在换算后点云坐标系中的 z）
///   或直接 g2p5.esti_floor = true 由每帧点云自动拟合地面（车辆系下地面法向确实朝 +z）。
////////////////////////////////////////////////////////////////////////////////////////////////////

Mat3d G2P5::R_vehicle_lidar() const { return vehicle_frame::R_vehicle_imu() * T_imu_lidar_.so3().matrix(); }

SE3 G2P5::ToWorkingPose(const SE3 &T_w_body) const {
    /// 个人修改：IMU 系世界位姿 → 车辆系世界位姿（位置向量与坐标系无关，只换旋转，见 vehicle_frame.h:189）
    return vehicle_frame::ToVehiclePose(T_w_body);
}

CloudPtr G2P5::ToWorkingCloud(const Keyframe::Ptr &kf) const {
    const Mat3d R = R_vehicle_lidar();
    const Vec3d t = vehicle_frame::R_vehicle_imu() * T_imu_lidar_.translation();

    const CloudPtr in = kf->GetCloud();
    CloudPtr out(new PointCloudType);
    if (in == nullptr) {
        return out;
    }

    out->reserve(in->size());
    for (const auto &pt : in->points) {
        const Vec3d p = R * Vec3d(pt.x, pt.y, pt.z) + t;

        PointType q;
        q.x = static_cast<float>(p.x());
        q.y = static_cast<float>(p.y());
        q.z = static_cast<float>(p.z());
        q.intensity = pt.intensity;
        q.time = pt.time;
        out->points.emplace_back(q);
    }

    out->width = out->points.size();
    out->height = 1;
    out->is_dense = false;
    out->header = in->header;
    return out;
}

void G2P5::RenderFront(Keyframe::Ptr kf) {
    {
        UL lock(frontend_mutex_);
        frontend_current_ = kf;
    }

    {
        UL lock{newest_map_mutex_};

        lightning::Timer::Evaluate([&]() { AddKfToMap({kf}, frontend_map_); }, "G2P5 Occupancy Mapping", true);
        newest_map_ = frontend_map_;
    }

    /// 向外回调
    if (map_update_cb_) {
        map_update_cb_(newest_map_);
    }
}

void G2P5::RedrawGlobalMap() { backend_redraw_flag_ = true; }

void G2P5::RenderBack() {
    while (!quit_flag_) {
        while (!backend_redraw_flag_ && !quit_flag_) {
            sleep(1);
        }

        if (quit_flag_) {
            break;
        }

        is_busy_ = true;

        /// 后端重绘被触发
        backend_redraw_flag_ = false;

        /// 重新绘制整张地图，如果中间过程又被重绘了，则退出
        std::vector<Keyframe::Ptr> all_keyframes;
        {
            UL lock(kf_mutex_);
            all_keyframes = all_keyframes_;
        }

        if (all_keyframes.empty()) {
            is_busy_ = false;
            continue;
        }

        G2P5Map::Options opt;
        opt.resolution_ = options_.grid_map_resolution_;
        backend_map_ = std::make_shared<G2P5Map>(opt);
        auto cur_kf = all_keyframes.begin();
        bool abort = false;

        for (; cur_kf != all_keyframes.end(); ++cur_kf) {
            AddKfToMap({*cur_kf}, backend_map_);
            if (backend_redraw_flag_) {
                LOG(INFO) << "backend redraw triggered in process, abort";
                abort = true;
                break;
            }

            if (quit_flag_) {
                abort = true;
                break;
            }
        }

        if (abort) {
            /// 继续重绘
            is_busy_ = false;
            continue;
        }

        /// 绘制过程中前端可能发生了更新，要保证后端绘制和前端的一致性
        int cur_idx = all_keyframes.back()->GetID();
        while (true) {
            Keyframe::Ptr frontend_kf = nullptr;
            {
                UL lock(frontend_mutex_);
                frontend_kf = frontend_current_;
            }

            if (cur_idx == frontend_kf->GetID()) {
                break;
            }

            int frontend_idx = frontend_kf->GetID();
            std::vector<Keyframe::Ptr> kfs;

            {
                UL lock(kf_mutex_);
                for (int i = cur_idx + 1; i <= frontend_idx; ++i) {
                    kfs.emplace_back(all_keyframes_[i]);
                }
            }

            AddKfToMap(kfs, backend_map_);
            cur_idx = frontend_idx;
        }

        {
            /// 同步前后端地图，替换newest map
            UL lock{newest_map_mutex_};
            frontend_map_ = backend_map_;
            newest_map_ = frontend_map_;
        }

        /// 向外回调
        if (map_update_cb_) {
            map_update_cb_(newest_map_);
        }

        is_busy_ = false;
    }

    LOG(INFO) << "backend render quit";
}

bool G2P5::ResizeMap(const std::vector<Keyframe::Ptr> &kfs, G2P5MapPtr &map) {
    /// 重设地图大小
    float init_min_x, init_min_y, init_max_x, init_max_y;
    float min_x, min_y, max_x, max_y;
    map->GetMinAndMax(init_min_x, init_min_y, init_max_x, init_max_y);
    min_x = init_min_x;
    min_y = init_min_y;
    max_x = init_max_x;
    max_y = init_max_y;

    /// 从后往前迭代
    for (auto it = kfs.begin(); it != kfs.end(); ++it) {
        if (quit_flag_) {
            return true;
        }

        auto kf = *it;

        /// 个人修改：统一换算到车辆(base_link)系后再做范围计算（见文件上方坐标系说明）
        SE3 pose = ToWorkingPose(kf->GetOptPose());  // 位姿：【IMU 系】→【车辆系】
        CloudPtr cloud = ToWorkingCloud(kf);         // 点云：【雷达系】→【车辆系】

        if (pose.translation().x() < min_x) {
            min_x = pose.translation().x();
        }

        if (pose.translation().y() < min_y) {
            min_y = pose.translation().y();
        }

        if (pose.translation().x() > max_x) {
            max_x = pose.translation().x();
        }

        if (pose.translation().y() > max_y) {
            max_y = pose.translation().y();
        }

        if (min_x > max_x || min_y > max_y) {
            return false;
        }

        for (size_t i = 0; i < cloud->points.size(); i += 10) {
            float range = cloud->points[i].getVector3fMap().norm();

            if (range > options_.usable_scan_range_ || range <= 0.01 || std::isnan(range)) {
                continue;
            }

            Vec3d point = pose * cloud->points[i].getVector3fMap().cast<double>();

            if ((point.x() - 1) < min_x) {
                min_x = point.x() - 1;
            }
            if ((point.y() - 1) < min_y) {
                min_y = point.y() - 1;
            }
            if ((point.x() + 1) > max_x) {
                max_x = point.x() + 1;
            }
            if ((point.y() + 1) > max_y) {
                max_y = point.y() + 1;
            }
        }
    }

    if (min_x > max_x || min_y > max_y) {
        return false;
    }

    /// resize map if necessary
    if (min_x < init_min_x || min_y < init_min_y || max_x > init_max_x || max_y > init_max_y) {
        min_x = (min_x > init_min_x) ? init_min_x : min_x;
        min_y = (min_y > init_min_y) ? init_min_y : min_y;
        max_x = (max_x < init_max_x) ? init_max_x : max_x;
        max_y = (max_y < init_max_y) ? init_max_y : max_y;

        float r = map->GetGridResolution();
        min_x = static_cast<int>((floor)(min_x / r)) * r;
        min_y = static_cast<int>((floor)(min_y / r)) * r;
        max_x = static_cast<int>((ceil)(max_x / r)) * r;
        max_y = static_cast<int>((ceil)(max_y / r)) * r;
        map->Resize(min_x, min_y, max_x, max_y);

        // LOG(INFO) << "map resized to " << min_x << ", " << min_y << ", " << max_x << ", " << max_y;
    }

    return true;
}

bool G2P5::AddKfToMap(const std::vector<Keyframe::Ptr> &kfs, G2P5MapPtr &map) {
    /// 如果需要，更新地图大小
    ResizeMap(kfs, map);

    for (const auto &kf : kfs) {
        Convert3DTo2DScan(kf, map);
    }

    return true;
}

G2P5MapPtr G2P5::GetNewestMap() {
    UL lock{newest_map_mutex_};
    LOG(INFO) << "getting newest map";
    if (newest_map_ == nullptr) {
        return nullptr;
    }

    return newest_map_->MakeDeepCopy();
}

void G2P5::Convert3DTo2DScan(Keyframe::Ptr kf, G2P5MapPtr &map) {
    /// 个人修改：先把点云从【雷达系】、位姿从【IMU 系】统一换算到【车辆(base_link)系】
    ///   (g2p5 的工作坐标系：x=前, y=左, z=上)。不做这一步，3D→2D 的"地面/极角"会算在
    ///   错误的平面里，栅格图整体错乱（详细推导见文件上方坐标系说明）。
    const CloudPtr cloud_vehicle = ToWorkingCloud(kf);
    const SE3 Twb = ToWorkingPose(kf->GetOptPose());

    // 3D转2D算法
    if (options_.esti_floor_) {
        if (!DetectPlaneCoeffs(cloud_vehicle)) {
            /// 如果动态检测失败，就用之前的参数
            floor_coeffs_ = Vec4d(0, 0, 1, -options_.default_floor_height_);
        } else {
            if (options_.verbose_) {
                LOG(INFO) << "floor coeffs: " << floor_coeffs_.transpose();
            }
        }
    } else {
        floor_coeffs_ = Vec4d(0, 0, 1, -options_.default_floor_height_);
    }

    // step 1. 计算每个方向上发出射线上的高度分布, // NOTE 转成整形的360度是有精度损失的
    std::vector<std::map<double, double>> rays(360);               // map键值：距离-相对高度（以距离排序）
    std::vector<Vec2d> angle_distance_height(360, Vec2d::Zero());  // 每个角度上的距离-高度值
    std::vector<Vec3d> pts_3d;  /// 距离地面 min_th_floor ~ max_th_floor 之间的点云，【车辆(base_link)系】下

    double min_th = options_.min_th_floor_;
    double max_th = options_.max_th_floor_;

    /// 个人修改：点云/位姿都已换算到车辆系，所以这里的 floor_coeffs_（地面 z=常数）、
    ///   极角 atan2(y,x)（水平面内）、以及 p_world = Twb * pc 三者才彼此自洽。
    const CloudPtr &cloud = cloud_vehicle;

    CloudPtr obstacle_cloud(new PointCloudType);

    /// 黑色点的处理方式：所有在障碍物范围内的都是黑色点
    int cnt_valid = 0;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto &pt = cloud->points[i];
        if (quit_flag_) {
            return;
        }

        Vec3d pc = Vec3d(pt.x, pt.y, pt.z);
        Vec4d pn = Vec4d(pt.x, pt.y, pt.z, 1);

        // 计算激光点所在角度方向以及高度
        Vec2d p = pc.head<2>();
        double dis = p.norm();

        if (dis > options_.usable_scan_range_) {
            continue;
        }

        double dis_floor = pn.dot(floor_coeffs_);  /// 该点到地面的距离
        double dangle = atan2(p[1], p[0]) * constant::kRAD2DEG;
        int angle = int(round(dangle) + 360) % 360;

        if (dis_floor > min_th) {
            if (dis_floor < max_th) {
                // 特别矮的和特别高的都不计入
                pts_3d.emplace_back(pc);

                rays[angle].insert({dis, dis_floor});

                /// 设置黑点
                Vec3d p_world = Twb * pc;
                map->SetHitPoint(p_world[0], p_world[1], true, dis_floor);

                cnt_valid++;

                obstacle_cloud->points.push_back(pt);
            }
        } else if (dis_floor > -min_th) {
            // 地面附近或者地面以下
            rays[angle].insert({dis, dis_floor});
            cnt_valid++;
        }
    }

    // if (options_.verbose_) {
    //     obstacle_cloud->is_dense = false;
    //     obstacle_cloud->height = 1;
    //     obstacle_cloud->width = obstacle_cloud->size();

    //     pcl::io::savePCDFile("./data/obs.pcd", *obstacle_cloud);
    // }

    // LOG(INFO) << "valid obs: " << cnt_valid << ", total: " << cloud->size();

    std::vector<double> floor_esti_data;  // 地面高度估计值

    // step 2, 考察每个方向上的分布曲线
    // 正常场景中，每个方向由较低高度开始（地面），转到较高的高度（物体）
    // 如若不是，那么可能发生了遮挡或进入盲区
    constexpr double default_ray_distance = -1;
    const double floor_rh = floor_coeffs_[3];

    for (int i = 0; i < 360; ++i) {
        if (quit_flag_) {
            return;
        }

        if (rays[i].size() < 2) {
            // 该方向测量数据很少，在16线中是不太可能出现的（至少地面上应该有线），出现，则说明有严重遮挡或失效，认为该方向取一个默认的最小距离（车宽）
            angle_distance_height[i] = Vec2d(default_ray_distance, floor_rh);
            continue;
        }

        /// 取距离和高度
        for (auto iter = rays[i].rbegin(); iter != rays[i].rend(); ++iter) {
            if (iter->second < options_.min_th_floor_) {
                angle_distance_height[i] = Vec2d(iter->first, iter->second);
                continue;
            }

            auto next_iter = iter;
            next_iter++;

            if (next_iter != rays[i].rend()) {
                if (iter->second > options_.min_th_floor_ && next_iter->second < options_.min_th_floor_) {
                    // 当前点是障碍但下一个点不是
                    angle_distance_height[i] = Vec2d(iter->first, iter->second);
                    break;
                }
            } else {
                angle_distance_height[i] = Vec2d(iter->first, iter->second);
            }
        }
    }

    // 以2D scan方式添加白色点
    SetWhitePoints(angle_distance_height, kf, map);
}

void G2P5::SetWhitePoints(const std::vector<Vec2d> &pt2d, Keyframe::Ptr kf, G2P5MapPtr &map) {
    assert(pt2d.size() == 360);

    /// 个人修改：pt2d 里的 (距离,相对地面高度) 来自【车辆(base_link)系】点云，
    ///   因此这里的位姿也必须用车辆系世界位姿（原来是【IMU 系】，会让"车到障碍物"的射线整体歪 90°）
    SE3 pose = ToWorkingPose(kf->GetOptPose());
    Vec3d orig = pose.translation();

    for (int i = 0; i < 360; ++i) {
        if (quit_flag_) {
            return;
        }

        double angle = float(i) * constant::kDEG2RAD;
        float r = pt2d[i][0];
        float h = pt2d[i][1];

        Vec3d p_local(r * cos(angle), r * sin(angle), h);
        Vec3d p_world = pose * p_local;

        /// 某方向无测量值时，认为无效
        if (r <= 0 || r > options_.usable_scan_range_) {
            /// 比较近时，涂白
            if (r < 0.1) {
                map->SetMissPoint(p_world[0], p_world[1], orig[0], orig[1], h, options_.lidar_height_);
            }
            continue;
        }

        map->SetMissPoint(p_world[0], p_world[1], orig[0], orig[1], h, options_.lidar_height_);
    }
}

bool G2P5::DetectPlaneCoeffs(const CloudPtr &input_cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.25);

    /// 个人修改：入参必须是已换算到【车辆(base_link)系】(z 向上) 的点云，
    ///   否则"地面法向 ≈ +z"这个判据（coefficients->values[2] > 0.99）不成立。
    CloudPtr cloud(new PointCloudType);
    for (auto &pt : input_cloud->points) {
        if (pt.z < options_.lidar_height_ + options_.default_floor_height_) {
            cloud->points.push_back(pt);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    if (cloud->size() < 200) {
        // LOG(ERROR) << "not enough points cloud->size(): " << cloud->size();
        return false;
    }

    // if (options_.verbose_) {
    //     pcl::io::savePCDFile("./data/floor_candi.pcd", *cloud);
    // }

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (coefficients->values[2] < 0.99) {
        LOG(ERROR) << "floor is not horizontal. ";
        return false;
    }

    if (inliers->indices.size() < 100) {
        LOG(ERROR) << "cannot get enough points on floor: " << inliers->indices.size();
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        floor_coeffs_[i] = coefficients->values[i];
    }
    cloud->clear();

    return true;
}

void G2P5::Init(std::string yaml_path) {
    auto yaml = YAML::LoadFile(yaml_path);
    options_.esti_floor_ = yaml["g2p5"]["esti_floor"].as<bool>();
    options_.min_th_floor_ = yaml["g2p5"]["min_th_floor"].as<float>();
    options_.max_th_floor_ = yaml["g2p5"]["max_th_floor"].as<float>();
    options_.lidar_height_ = yaml["g2p5"]["lidar_height"].as<float>();
    options_.grid_map_resolution_ = yaml["g2p5"]["grid_map_resolution"].as<float>();
    options_.default_floor_height_ = yaml["g2p5"]["floor_height"].as<float>();

    /// 个人修改：读取"雷达系 → IMU 系"外参（与 laser_mapping.cc 读的是同一组 yaml 键），
    ///   用于把【雷达系】关键帧点云换算到 g2p5 的工作坐标系（车辆/base_link 系），
    ///   否则栅格地图整体歪 90°（详见文件上方坐标系说明）。
    {
        const auto ext_t = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        const auto ext_r = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        if (ext_r.size() == 9 && ext_t.size() == 3) {
            Mat3d R;
            R << ext_r[0], ext_r[1], ext_r[2],  //
                ext_r[3], ext_r[4], ext_r[5],  //
                ext_r[6], ext_r[7], ext_r[8];
            /// 个人修改（2026-10-02，第 4 次修复③ 配套）：yaml 里手写的 9 个数带 ~1e-9 非正交误差，
            ///   直接 SO3(R) 会被 Sophus 的 isOrthogonal() 前置检查 abort（见 lightning_math.hpp 注释），
            ///   故先正交化（输入本就正交时原样返回，不改变既有行为）。
            R = math::OrthonormalizeRotation(R);
            T_imu_lidar_ = SE3(SO3(R), Vec3d(ext_t[0], ext_t[1], ext_t[2]));
            LOG(INFO) << "个人修改 g2p5：雷达->IMU 外参已读入, t = " << T_imu_lidar_.translation().transpose()
                      << ", R =\n"
                      << T_imu_lidar_.so3().matrix();
        } else {
            LOG(ERROR) << "个人修改 g2p5：fasterlio.extrinsic_R/T 维度不对 (R:" << ext_r.size()
                       << ", T:" << ext_t.size() << ")，按单位阵处理（栅格地图可能整体歪 90°）";
            T_imu_lidar_ = SE3();
        }
    }

    G2P5Map::Options opt;
    opt.resolution_ = options_.grid_map_resolution_;
    frontend_map_ = std::make_shared<G2P5Map>(opt);

    if (options_.online_mode_) {
        draw_frontend_map_thread_.SetProcFunc([this](Keyframe::Ptr kf) { RenderFront(kf); });
        draw_frontend_map_thread_.Start();
    }

    draw_backend_map_thread_ = std::thread([this]() { RenderBack(); });
}

}  // namespace lightning::g2p5