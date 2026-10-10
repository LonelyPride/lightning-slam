#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <deque>
#include <functional>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/options.h"
#include "core/ivox3d/ivox3d.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_processing.hpp"
#include "pointcloud_preprocess.h"

#include "livox_ros_driver2/msg/custom_msg.hpp"

// 个人修改：SetUI() 里需要调用 PangolinWindow::SetTImuLidar()，因此这里必须拿到完整类型
// （pangolin_window.h 只包含 common 下的轻量头文件，不会引入 OpenGL/pangolin）
#include "ui/pangolin_window.h"

namespace lightning {

/**
 * laser mapping
 * 目前有个问题：点云在缓存之后，实际处理的并不是最新的那个点云（通常是buffer里的前一个），这是因为bag里的点云用的开始时间戳，导致
 * 点云的结束时间要比IMU多0.1s左右。为了同步最近的IMU，就只能处理缓冲队列里的那个点云，而不是最新的点云
 */
class LaserMapping {
   public:
    struct Options {
        Options() {}

        bool is_in_slam_mode_ = true;  // 是否在slam模式下

        bool enable_icp_part_ = true;    // 是否添加ICP部分
        double plane_icp_weight_ = 1.0;  // 点面ICP部分的权重
        double icp_weight_ = 100;        // ICP部分的权重

        int min_pts = 300;  // 配准所需的点数

        // 关键帧阈值
        double kf_dis_th_ = 2.0;
        double kf_angle_th_ = 15 * M_PI / 180.0;

        bool proj_kfs_ = false;
        int max_proj_kfs_ = 5;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping(Options options = Options());
    ~LaserMapping() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "laser mapping deconstruct";
    }

    // init without ros
    bool Init(const std::string &config_yaml);

    bool Run();

    // callbacks of lidar and imu
    // 处理ROS2的点云
    void ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    // 处理livox的点云
    void ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);

    // 如果已经做了预处理，也可以直接处理点云
    void ProcessPointCloud2(CloudPtr cloud);

    void ProcessIMU(const lightning::IMUPtr &msg_in);

    // 保存前端的地图
    void SaveMap();

    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) {
        ui_ = ui;

        // 个人修改：把"雷达系 → IMU 系"外参交给 UI。
        // 原因：UI 收到的点云都是【雷达系】（去畸变时 imu_processing.hpp 已用 R_lidar_imu_ᵀ 把点转回雷达系），
        //       而 UI 收到的姿态是 IMU/车辆系，UI 必须知道此外参才能把点云摆到地图的正确位置。
        // 注：offset_R_lidar_fixed_/offset_t_lidar_fixed_ 在 Init()→LoadParamsFromYAML() 中已读入，
        //     而 SetUI() 一定在 Init() 之后调用（见 slam.cc）。
        if (ui_ != nullptr) {
            ui_->SetTImuLidar(SE3(offset_R_lidar_fixed_, offset_t_lidar_fixed_));
        }
    }

    // 获取关键帧
    Keyframe::Ptr GetKeyframe() const { return last_kf_; }

    // 获取激光的状态
    NavState GetState() const { return state_point_; }

    // 个人修改（方案C）：世界系被"重力再对齐"（刚性旋转）时的回调，供 slam.cc 通知 g2p5 重绘栅格图。
    //   说明：再对齐会把已有关键帧位姿整体旋转 ⇒ 在线已经画进 g2p5 栅格图的点会留在旧世界系里，
    //   必须让 g2p5 用新位姿重绘（G2P5::RedrawGlobalMap() 就是"按当前关键帧位姿重建整张图"）。
    void SetWorldFrameFixedCallback(std::function<void()> cb) { world_frame_fixed_cb_ = cb; }

    // 个人修改（方案C）：把"IMU 静止窗口判定阈值"透传给 ImuProcess（yaml 可选键驱动）
    void SetImuInitStaticParams(double acc_tol, double gyr_tol, double std_rel_tol, double min_duration_s,
                                double max_duration_s, int max_wait_count) {
        p_imu_->SetInitStaticParams(acc_tol, gyr_tol, std_rel_tol, min_duration_s, max_duration_s, max_wait_count);
    }

    // 获取IMU状态
    NavState GetIMUState() const {
        if (p_imu_->IsIMUInited()) {
            return kf_imu_.GetX();
        } else {
            NavState s;
            s.pose_is_ok_ = false;
            return s;
        }
    }

    CloudPtr GetScanUndist() const { return scan_undistort_; }
    CloudPtr GetProjCloud();

    // 获取最新的点云
    CloudPtr GetRecentCloud();

    std::vector<Keyframe::Ptr> GetAllKeyframes() { return all_keyframes_; }

    /**
     * 计算全局地图
     * @param use_lio_pose
     * @return
     */
    CloudPtr GetGlobalMap(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

    // 个人修改："雷达系 → IMU 系"外参（p_imu = R * p_lidar + t），即 yaml 里的 extrinsic_R/extrinsic_T。
    // 供 UI 绘制雷达系点云时使用（T_w_lidar = T_w_body * T_imu_lidar）。
    // 个人修改：改为 public —— 地图保存（slam.cc 写 index.txt 的 start 功能点）等【模块外】代码
    //   也必须用同一组外参把 IMU 系位姿换算成雷达系位姿，避免各处重复解析 yaml 造成不一致。
    SE3 T_imu_lidar_extrinsic() const { return SE3(offset_R_lidar_fixed_, offset_t_lidar_fixed_); }

   private:
    // sync lidar with imu
    bool SyncPackages();

    void ObsModel(NavState &s, ESKF::CustomObservationModel &obs);

    // 个人修改（方案C）：把"重力方向"作为姿态观测（ESKF::ObsType::ACC_AS_GRAVITY 的观测函数）。
    //   静止/低动态时 IMU 测到的比力方向就是"上"，用它给姿态提供绝对竖直参考：
    //     · 消除"姿态被 LiDAR 更新冻结后再也无法纠回"的缺陷（世界系倾斜 / z 持续下漂的根因之一）；
    //     · HTH 天然秩2（绕重力方向的自转不可观测），作为【软约束】叠加在 LiDAR 观测之上，
    //       权重由 fasterlio.gravity_obs_sigma_deg 决定（默认 1.5°），不会与 LiDAR 抢主导权。
    void AccAsGravityObsModel(NavState &s, ESKF::CustomObservationModel &obs);

    // 个人修改（方案C）：当前 scan 窗口内的 IMU 均值（比力已乘量纲因子、角速度已扣 bg）。
    //   为什么要用【窗口均值】而不是最后一个采样：真实 IMU 的单采样噪声会让 |ω| 轻易超过门限
    //   （实测单采样可达 0.24 rad/s），导致重力观测几乎每帧都被"低动态门限"判掉、完全不生效；
    //   窗口（≈0.1 s、10~20 个采样）均值可把噪声降到 1/4 左右。
    bool GetWindowImuMean(Vec3d *acc_mean, Vec3d *gyr_mean, double *acc_dir_spread_deg = nullptr) const;

    // 个人修改（方案C）：低动态样本的重力方向累积（世界系），用于在线自检与世界系重力再对齐
    void AccumulateGravityDirection(const NavState &s);

    // 个人修改（方案C，2026-10-02 第3次加固）：用【LIO 世界系速度差分】估计真实平动加速度（m/s²）。
    //   每帧调用一次（入口 = AccumulateGravityDirection），结果放在 motion_acc_est_ / motion_acc_valid_，
    //   供"自检"与"世界系再对齐累积"的平动加速度门限使用（见头文件里门限参数的说明）。
    void UpdateMotionAccEstimate(const NavState &s);

    // 个人修改（方案C）：世界系重力再对齐 —— 一旦发现已建好的世界系与重力明显不符，
    //   把位姿/关键帧/局部地图整体刚性旋转回"z 轴竖直向上"（等价于换一个世界系原点基准，
    //   不引入任何形变），从而把倾斜的地图/轨迹一次性纠回来。
    void TryRealignWorldFrameToGravity();

    // 个人修改（方案C）：用当前关键帧位姿重建 iVox 局部地图（世界系被旋转后必须重建）
    void RebuildLocalMapFromKeyframes();

    // 个人修改（方案C）：打印/检查"姿态所定义的世界系 z 轴"与实测重力的夹角（度），返回该夹角
    double CheckGravityAlignment(const NavState &s, bool print);

    inline void PointBodyToWorld(const PointType &pi, PointType &po) {
        Vec3d p_global(state_point_.rot_ *
                           (offset_R_lidar_fixed_ * pi.getVector3fMap().cast<double>() + offset_t_lidar_fixed_) +
                       state_point_.pos_);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();

    bool LoadParamsFromYAML(const std::string &yaml);

    // 创建关键帧
    void MakeKF();

    // 将附近的关键帧投影至cloud中
    void ProjectKFs(CloudPtr cloud, int size_limit = 1000);

   private:
    Options options_;

    // modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    // local map related
    double filter_size_map_min_ = 0;

    // params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    Mat3d offset_R_lidar_fixed_ = Mat3d::Identity();
    Vec3d offset_t_lidar_fixed_ = Vec3d::Zero();
    std::string map_file_path_;

    std::vector<Keyframe::Ptr> all_keyframes_;
    Keyframe::Ptr last_kf_ = nullptr;
    int kf_id_ = 0;

    // point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan

    // 点面相关
    std::vector<PointVector> nearest_points_;  // nearest points of current scan
    std::vector<Vec4f> corr_pts_;              // inlier pts
    std::vector<Vec4f> corr_norm_;             // inlier plane norms
    std::vector<float> residuals_;             // point-to-plane residuals
    std::vector<char> point_selected_surf_;    // selected points
    std::vector<Vec4f> plane_coef_;            // plane coeffs

    // 点到点相关
    std::vector<char> point_selected_icp_;  // 点到点的selected points

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;

    // options
    bool keep_first_imu_estimation_ = false;  // 在没有建立地图前，是否要使用前几帧的IMU状态
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    bool enable_skip_lidar_ = true;  // 雷达是否需要跳帧
    int skip_lidar_num_ = 5;         // 每隔多少帧跳一个雷达
    int skip_lidar_cnt_ = 0;

    // statistics and flags //
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_surf_ = 0, frame_num_ = 0, effect_feat_icp_ = 0;

    double last_lidar_time_ = 0;

    ///////////////// EKF inputs and output /////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan

    ESKF kf_;      // 点云时刻的IMU状态
    ESKF kf_imu_;  // imu 最新时刻的eskf状态

    NavState state_point_;  // ekf current state

    // 个人修改（方案A+）：IMU → 车辆(base_link) 安装旋转的配置（yaml 的 common 段）
    bool vehicle_imu_auto_ = false;               // vehicle_imu_mount: auto ⇒ 等 IMU 初始化后用重力自动对齐
    bool vehicle_imu_ready_ = false;              // 是否已确定安装旋转（避免重复设置）
    std::string vehicle_imu_forward_axis_ = "z";  // auto 模式下"车头"对应 IMU 的哪个轴（x/y/z，可带负号）

    // 个人修改（方案A+）：auto 模式下用 IMU 静止初始化的重力方向确定 IMU→车辆 安装旋转。
    //   在 Run() 里每帧调用，仅在 IMU 首次初始化完成后生效一次（随后 vehicle_imu_ready_ 置位）。
    void TryInitVehicleFrameFromGravity();

    // =============================================================================================
    //  个人修改（2026-10-02，方案C：姿态/重力根治）—— 姿态的绝对竖直参考 + 世界系自检/再对齐
    //
    //  【要解决的问题】原实现里姿态只有 IMU 陀螺积分与 LiDAR 点面观测两种信息：
    //    · 陀螺积分没有绝对参考；
    //    · LiDAR 观测在长直/空旷/单面墙等几何退化场景下对 roll/pitch 约束很弱，
    //      于是姿态会被慢慢"拉歪"，而一旦 P 收缩，任何观测都很难把它纠回来。
    //    结果：世界系整体倾斜（实测本 bag 24.32°），车明明水平，z 却沿路径持续下降
    //    （地图地面在 3.7 m 内下降 1.12 m，斜率 0.404 ≈ 22°）。
    //
    //  【本组参数的作用】
    //    gravity_obs_*      ：把"低动态时的加计方向 = 竖直向上"作为姿态软观测（每个 scan 一次）
    //    world_frame_realign_*：若已建好的世界系与重力明显不符，在建图早期把整个世界系
    //                          （位姿+关键帧+局部地图）刚性旋转回竖直 —— 一次性纠正已倾斜的地图
    // =============================================================================================
    bool gravity_obs_enable_ = true;            // 是否启用重力/竖直姿态观测
    double gravity_obs_sigma_deg_ = 1.5;        // 观测噪声（度）：1.5° ⇒ 权重 1/σ²，远弱于 LiDAR，仅做慢修正
    double gravity_obs_acc_tol_ = 0.15;         // 低动态门限：| |a|/g - 1 | 需小于此值（0.15 ⇒ 1.5 m/s²）
    double gravity_obs_gyr_tol_ = 0.15;         // 低动态门限：|ω| 需小于此值（rad/s，≈8.6°/s；对窗口均值判定）
    // 个人修改（方案C，2026-10-02 加固）：低动态门限 —— 窗口内【比力方向相对窗口均值的最大夹角】（度）。
    //   为什么必须有它：横向/冲击加速度会让 |a| 几乎不变（幅值判据放过），但方向偏离竖直 10~15°。
    //   本 bag 实测：|a|=8.68 m/s²（仅偏 1.13 m/s²）时方向偏竖直 15.33°
    //   ⇒ 只看幅值时，这个"假重力"会被喂给重力观测/自检，把车体姿态也一起带歪（日志里 d_rot 忽大忽小）。
    //   3.0° ≈ 0.5 m/s² 的平动冲击，正常行驶远小于它；设为 180 可关闭本判据。
    double gravity_obs_acc_dir_tol_deg_ = 3.0;
    // 个人修改（方案C，2026-10-02 第2次加固）：自检/世界系再对齐判据的【ba 一致性告警阈值】（度）。
    //   背景（本 bag 实测）：ESKF 的加计零偏 ba 会被机动"拽"到 |ba| ≈ 1.2 m/s²（≈7°），
    //   而 ba 观测性极差（b_acc_cov 小 ⇒ 学到后几乎不回弹）⇒ 被污染后长期保持错值。
    //   此时用 (a - ba) 算出的"世界系与重力夹角"会变成 4.9~12.9° 的假警报
    //   （同一时刻用"地图地面倾角 21.05° vs LiDAR+加计物理实测 22.77°"交叉验证，世界系其实只歪 ~1.7°）；
    //   更危险的是【世界系再对齐】也拿这个数做依据 —— 本 bag 差 1.6 ms 就真的把地图整体旋转 12.9°。
    //   ⇒ 判据改用【未扣 ba 的原始比力方向】（准静态窗口里 |a| ≈ g ⇒ 它本身就等于重力方向，与 ba 无关），
    //     并在 ba 等效倾角超过本阈值时打印告警，提醒"带 ba 的数字不可当世界系倾斜用"。
    double gravity_obs_ba_warn_deg_ = 2.0;
    int gravity_obs_every_n_scan_ = 1;          // 每 N 个 scan 施加一次（1 = 每帧）
    double gravity_obs_sigma_scale_ = 1.0;      // 权重整体缩放（调试用，1.0 = 按上面 sigma 计算）
    // 个人修改（方案C）：施加重力观测【之前】给姿态协方差充气的标准差（度）。
    //   为什么必须要有它：LiDAR 观测每帧都把姿态协方差压到极小（信息量 ~5e6 ⇒ σ≈0.03°），
    //   此时 1.5° 的"软观测"增益只有 ~1e-4 ⇒ 单帧修正量 1e-4 度，等于完全不起作用
    //   （实测日志：世界系已歪 20°，而 d_rot 只有 0.0001°）。
    //   充气到 0.5° 后，增益 ≈ 0.25/(0.25+2.25) ≈ 0.1，即每帧修正当前偏差的 10%
    //   （时间常数 ~1 s），既能真正把姿态拉向竖直，又因为只修正 10% 而绝不会突变/振荡。
    double gravity_obs_prior_inflation_deg_ = 0.5;

    // 个人修改（方案C，2026-10-02 第4次修复①）：【行驶中的竖直锚】（本次真正的 roll 漂移修复）。
    //   问题（实测证据）：重力观测只在"低动态"时施加，而行驶中【窗口内比力方向散布】几乎恒超过 3°
    //   （日志 `gravity obs skipped (dynamic): |a|/g-1 = 0.0057, |w| = 0.0047 rad/s, 方向散布 = 3.20 deg`
    //     ⇒ ESKF 里 `valid_=false` 会把这次更新整体回滚，日志表现为 `d_rot = 0 deg` 而同一时刻
    //     "世界系夹角(无ba) = 4.77°"），于是【车一开动，姿态就没有任何绝对竖直参考】：
    //     陀螺零偏（本 bag 实测 bg_z = 0.0416 rad/s = 2.39°/s，正好是 roll 轴）被 ESKF 允许在运动中
    //     重新随机游走（b_gyr_cov = 5e-4）⇒ roll 从 0 一路积到 +18.97°、世界系随之倾斜 ⇒
    //     z 沿路径"下弓" 1.37 m，停车后重力观测重新生效又把它拉回来（本 bag 实测 |z| 回到 0.026 m）。
    //   ⇒ 运动中也必须给竖直锚。判据换成"能证明【此刻比力 ≈ 纯重力】"的四项（本 bag 均有数据支撑）：
    //     ① | |a|/g - 1 | ≤ gravity_obs_acc_tol_（0.15 ⇒ 1.5 m/s²）
    //     ② |ω| ≤ gravity_obs_gyr_tol_（0.15 rad/s）
    //     ③ 【平动加速度】|Δv|/Δt ≤ gravity_obs_motion_acc_tol_（LIO 速度差分；**必须有效**）
    //        —— 这才是挡"持续加/减速、转弯侧向加速度制造的假重力"的真闸门：
    //           那种假重力 |a|/|ω|/方向散布全都看不出来（见 UpdateMotionAccEstimate 的说明）。
    //     ④ 窗口内比力方向散布 ≤ gravity_obs_motion_acc_dir_tol_deg_（默认放宽到 10°）：
    //        3° 的门限在行驶中几乎恒被撞穿（实测 3.20°），它反映的是路面振动，而振动是【随机】的
    //        （窗口均值的误差 ≪ 散布本身），系统性假重力已由③负责，故运动中放宽。
    //   权重仍远弱于 LiDAR（σ 默认 3.0°，是静止值 1.5° 的两倍）⇒ 只做"慢拉"，不抢 LiDAR 主导权，
    //   单帧增益 ≈ 0.5²/(0.5²+3²) ≈ 2.7%（配合 gravity_obs_prior_inflation_deg_）⇒ 时间常数 ~4 s。
    bool gravity_obs_motion_enable_ = true;
    double gravity_obs_motion_sigma_deg_ = 3.0;         // 行驶中重力观测的噪声(度)；越大越"软"
    double gravity_obs_motion_acc_dir_tol_deg_ = 10.0;  // 行驶中的比力方向散布门限(度)
    double gravity_obs_motion_min_speed_mps_ = 0.1;     // 判定"在动"的 LIO 速度门限(m/s)

    // 个人修改（方案C，2026-10-02 第4次修复③ / ★第5次修复更正）：【外参 up 方向自检】。
    //   启动瞬间比较"配置/标定声称的 IMU 上方向"与"实测竖直方向"（IMU 静止窗口均值 mean_acc）。
    //   ★第5次修复更正：参照向量必须是【雷达系实测路面法向】ground_normal_lidar_（yaml 可选键
    //     fasterlio.ground_normal_lidar），而不是硬编码的雷达 +y：本平台雷达单元相对重力是倾斜
    //     安装的，+y 与实测路面法向差 10.590° ⇒ 用 +y 当参照等价于把"路面"拧 10.59°，正是"地图
    //     地面/墙整体倾斜 10.6°、z 按 tan(10.6°)=0.187 m/m 漂移"的原因（第4次修复后 val_4th_b
    //     实测地面倾角 10.602°，与 angle(R·n_l, a0) = 10.59° 完全一致）。
    //   判据：angle(R·n_l, a0) ≤ 0.4° ⇒ 地图地面与世界系 z 一致（第5次修复后本平台实测 0.007°）。
    //   诊断量 mount_tilt_deg = angle(R·(+y), a0) ≈ 10.59°【属正常】：它是雷达单元相对重力的安装
    //     倾角，不是外参错误，不要据此去改 extrinsic_R。
    double extrinsic_up_check_deg_ = -1.0;  // 自检结果(度)；<0 表示尚未测出
    // 个人修改（2026-10-02 第5次修复）：雷达系里的【实测路面法向】（单位向量）——up 自检的参照。
    //   来源：yaml fasterlio.ground_normal_lidar（缺省 (0,1,0) = 旧行为）；标定方法见 yaml 注释。
    Vec3d ground_normal_lidar_ = Vec3d(0, 1, 0);

    // 个人修改（2026-10-09 第6次修复，方案D）：加计比力方向符号（yaml 可选键 fasterlio.imu_acc_up_sign）。
    //   +1 = 标准（静止读数 = +g·(上)，本工程 val 平台如此）；-1 = 该平台驱动把比力方向反了号（狗平台实测）。
    //   在 LoadParamsFromYAML 里读出后立即 p_imu_->SetAccUpSign()，laser_mapping 自身在
    //   GetWindowImuMean() 与两处 kf_imu_.Predict() 里用 p_imu_->GetAccUpSign() 取同一符号。
    double imu_acc_up_sign_ = 1.0;

    // 个人修改（方案C，2026-10-02 第4次修复①②）诊断计数（用于验收"行驶中是否真的在施加竖直锚"）
    int gravity_obs_last_mode_ = 0;         // 0=本次未施加, 1=静止窗口施加, 2=行驶中施加
    long gravity_obs_applied_static_ = 0;   // 静止窗口成功施加次数
    long gravity_obs_applied_motion_ = 0;   // 行驶中成功施加次数（验收要求 >0）
    long gravity_obs_skipped_ = 0;          // 被门限挡掉次数

    bool world_frame_realign_enable_ = true;    // 是否允许"世界系重力再对齐"
    bool world_frame_realigned_ = false;        // 是否已经做过（只做一次）
    double world_frame_realign_min_deg_ = 0.7;  // 小于此角度不做（避免无谓的整体旋转）
    double world_frame_realign_max_deg_ = 30.0; // 大于此角度不自动旋转，只告警（此时通常意味着数据本身有问题）
    int world_frame_realign_max_kf_id_ = 40;    // 只允许最初这么多个关键帧以内做（保证重建代价小、地图尚未成形）
    double world_frame_realign_max_time_s_ = 15.0;  // 只允许建图开始后这么久以内做
    int world_frame_realign_min_samples_ = 20;  // 至少累积这么多个低动态重力样本才敢动

    Vec3d gravity_dir_world_sum_ = Vec3d::Zero();  // 低动态重力方向（世界系）累加（未归一化）
    int gravity_dir_world_count_ = 0;              // 有效样本数
    double gravity_misalign_deg_ = 0.0;            // 最近一个被接受的静止样本的夹角(度)；均值在 TryRealign 里现算

    // 个人修改（方案C，2026-10-02 第3次加固）：【平动加速度门限】（m/s²）—— 由 LIO 世界系速度差分估计。
    //   为什么必须要有它（本 bag 实测证据）：加计测的是"比力" = 重力 + 平动加速度 ⇒ 【恒定的加/减速】
    //   会做出一个 15° 级的"假重力"，而前三个判据全都放过它：
    //     ① | |a|/g - 1 |：沿 22° 坡道以 2.6 m/s² 加速时 |a| 只变 ~3.5%（acc_tol = 0.15 放过）；
    //     ② |ω|：平稳加减速时角速度很小（gyr_tol = 0.15 rad/s 放过）；
    //     ③ 窗口内比力方向散布：加速度近似恒定时方向几乎不变（dir_tol = 3° 放过）。
    //   实测：t=59.8 s 这一次"准静态"自检读出 15.58° 的"世界系歪了"，而同一时刻车辆正从静止加速到
    //   0.3 m/s；对照 t=56.9 s（真正静止：|v| = 0.003 m/s、|ω| = 0.001 rad/s）读数只有 0.26°（且 ba≈0）
    //   ⇒ 那个大数字是平动加速度造成的，不是世界系倾斜。世界系再对齐会按累积值刚性旋转整张地图，
    //   一旦吃到这种样本就会把好地图转歪 15°（本 bag 上一轮差 1.6 ms 就会按 12.9° 转），所以必须剔掉。
    //   LIO 速度来自雷达/地图（与加计零偏无关）⇒ |Δv|/Δt 就是真实平动加速度。
    //   0.35 m/s² ≈ 2° 的比力方向误差；设成很大（如 100）即关闭本判据。
    double gravity_obs_motion_acc_tol_ = 0.35;
    double gravity_obs_motion_window_s_ = 1.0;   // 速度差分基线(s)：太短会被速度噪声淹没，太长会跨过机动

    // 个人修改（方案C，2026-10-02 第3次加固）：【静止门限】(m/s)——"世界系再对齐"只采信确认静止的样本。
    //   为什么还需要它（前面四项判据的最后一个漏洞）：平动加速度门限挡的是"持续的加/减速"，而
    //   【运动中过坎/坑、车身瞬时冲击/侧倾】这类样本：① 1 s 平均 |Δv| 可能很小（先加后减、净变化≈0）、
    //   ② |ω| 小、③ 窗口内方向散布小 ⇒ 能同时骗过 acc_tol / gyr_tol / acc_dir_tol / motion_acc_tol
    //   （本 bag 实测确有一个 15.58° 的样本通过了全部四项，而同一 bag 真静止时读数是 0.26°）。
    //   再对齐会【刚性旋转整张地图】⇒ 这种判定必须用"车真停着、比力就是重力"的铁样本。
    //   每个建图任务的起步段本来就有静止样本，够用。|v| 取 LIO 世界系速度（与加计无关）。
    //   设成很大（如 100）即关闭本判据（不建议；只在明确知道自己在做什么时改）。
    double gravity_obs_max_speed_mps_ = 0.05;

    std::deque<std::pair<double, Vec3d>> gravity_speed_hist_;  // (lidar_end_time, 世界系速度)
    double motion_acc_est_ = 0.0;                // 最近一次平动加速度估计（m/s²）
    bool motion_acc_valid_ = false;              // 该估计是否有效（速度历史不足一个基线时为 false）

    std::function<void()> world_frame_fixed_cb_;   // 世界系被旋转后的通知（g2p5 重绘）

    bool use_aa_ = false;  // use anderson acceleration?

    std::list<Keyframe::Ptr> proj_kfs_;  // 投影到当前帧的关键帧

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace lightning

#endif  // FASTER_LIO_LASER_MAPPING_H
