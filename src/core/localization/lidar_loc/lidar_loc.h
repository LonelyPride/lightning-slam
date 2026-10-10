#pragma once

#include <pcl/registration/icp.h>
#include <chrono>
#include <deque>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "common/nav_state.h"
#include "common/timed_pose.h"
#include "core/localization/localization_result.h"
#include "core/maps/tiled_map.h"

#include "pclomp/ndt_omp_impl.hpp"

namespace lightning::ui {
class PangolinWindow;
}

namespace lightning::loc {

// 激光定位对外接口类
class LidarLoc {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using UL = std::unique_lock<std::mutex>;

    // 定位方法
    enum class LocMethod {
        NDT_OMP,  // OMP版本NDT
    };

    struct Options {
        Options() {}

        bool display_realtime_cloud_ = false;          // 是否显示实时点云
        bool debug_ = false;                           // 是否使用单测模式
        LocMethod match_method_ = LocMethod::NDT_OMP;  // 匹配方式
        bool try_self_extrap_ = false;                 // 是否尝试自己的外推pose
        bool with_height_ = true;                      // 建图期间是否带有高度约束？
        bool force_2d_ = true;                         // 强制在2D空间
        float min_init_confidence_ = 0.1;              // 初始化时要求的最小分值
        bool init_with_fp_ = true;                     // 是否使用功能点进行初始化
        bool enable_parking_static_ = false;           // 是否在静止时输出固定位置
        bool enable_icp_adjust_ = false;               // 是否使用icp调整ndt匹配结果提高定位精度

        // 个人修改（2026-10-01）：手动初始位姿（RViz "2D Pose Estimate" / 外部话题）配套参数。
        //   手动点选天然带"点击延迟"（车在动时约 0.5~2 m）与朝向误差，而细分辨率 NDT 收敛半径小，
        //   直接当种子容易被 min_init_confidence_ 拒掉；故对【手动种子】额外做一步
        //   "粗分辨率(5m) NDT + 小范围 yaw 搜索"，再把结果交给细分辨率 NDT，最终仍由同一门槛把关。
        //   普通初始化（功能点 start / recover）不启用，行为与修改前一致。
        bool manual_init_rough_search_ = true;      // 是否启用手动种子的粗分辨率搜索
        double manual_init_yaw_search_deg_ = 30.0;  // yaw 搜索范围（度，种子左右各搜这么多）
        int manual_init_yaw_search_step_ = 7;       // yaw 搜索采样数（均匀分布在 ±范围内）
        int manual_init_refine_iter_ = 2;           // 接受手动种子后再迭代精修的次数（见 InitWithFP）

        // 个人修改（2026-10-02，方案C：z 漂移根治）—— 高度约束与地图健康度自检
        //
        // 【为什么要有高度约束】本工程定位端只有"点云匹配"一种绝对观测：
        //   匹配输出 z 完全由地图决定。当地图本身倾斜（建图时世界系没和重力对齐，
        //   本 bag 实测地图地面在 3.7 m 内下降 1.12 m ≈ 22°）时，匹配会忠实地"跟着地图走"，
        //   表现为定位 z 沿路径持续漂移 —— 定位端无法通过匹配修好它，只能：
        //     ① 把 z 约束到地图起点的高度附近（本组参数，原代码里被注释掉的 with_height_ 逻辑）；
        //     ② 启动时自检地图地面倾角并大声告警（CheckMapLevel），提示重建地图。
        //
        // 【语义】z_target = map_height_（首帧功能点所在高度）
        //     tol = 0        ⇒ 完全锁定（原注释代码的行为；路线含坡道时会在坡道段产生误差）
        //     tol > 0        ⇒ "软皮带"：只约束偏离超过 ±tol 的部分（默认 0.3 m）
        //     gain = 0       ⇒ 关闭（等价于修改前行为）
        bool height_constraint_enable_ = true;   // 总开关（缺省取 loop_closing.with_height）
        double height_constraint_gain_ = 1.0;    // 修正增益：z += gain * clamp(map_height_ - z, ±tol)
        double height_constraint_tol_ = 0.3;     // 软皮带半宽(m)；0 = 完全锁定

        // 个人修改（方案C）：地图地面健康度自检（Init 时按已加载地图做一次平面拟合）
        bool map_level_check_enable_ = true;     // 是否在 Init 时自检
        double map_level_warn_deg_ = 2.0;        // 地面相对水平面的倾角超过该值即 WARNING（建议重建地图）
        double map_level_radius_ = 25.0;         // 只统计起点附近该半径内的地面点（m）

        // 个人修改（方案C）：force_2d 时是否也把 z 归零（原行为），默认 false ——
        //   原来 force_2d 会把 z 直接置 0（那是"2D 导航、地图原点在地面"的布局假设）；
        //   本平台地图原点在 IMU 处、且路线可能有坡度，z 归零会让定位整体偏低。
        //   现在默认保留匹配出的 z（再由 height_constraint 约束），需要老行为时置 true。
        bool force_2d_zero_z_ = false;

        // 点云过滤
        // float filter_z_min_ = -1.0;
        float filter_z_max_ = 30.0;
        // float filter_intensity_min_ = 10.0;
        // float filter_intensity_max_ = 100.0;

        // 地图配置
        TiledMap::Options map_option_ = TiledMap::Options();

        // 动态图层更新相关
        bool update_dynamic_cloud_ = true;     // 是否使用定位后的点云来更新动态图层
        double update_kf_dis_ = 5.0;           // 每隔多少米更新一次
        double update_kf_time_ = 10.0;         // 每隔多少时间更新一次
        double update_lidar_loc_score_ = 2.2;  // 更新时激光定位匹配分值阈值
        double lidar_loc_odom_th_ = 0.3;       // 激光两帧匹配结果与对应lidarodom结果差的阈值，超过则认为lidarodom异常

        double max_update_cache_dis_ = 30.0;  // 更新动态图层的缓冲距离
        std::string recover_pose_path_ = "./data/recover_pose.txt";
    };

    explicit LidarLoc(Options options = Options());
    virtual ~LidarLoc();

    // 初始化
    bool Init(const std::string& config_path);

    // 处理 lidar odom 消息
    bool ProcessLO(const NavState& state);

    // 处理拼接后点云
    bool ProcessCloud(CloudPtr cloud_input);

    // 处理DR状态
    bool ProcessDR(const NavState& state);

    // 获取位姿
    NavState GetState();

    // 设置当前定位状态标志位
    void SetInitRltState();

    /**
     * 尝试在另一个位置进行定位
     * @param input
     * @param pose
     * @return
     */
    bool TryOtherSolution(CloudPtr input, SE3& pose);

    // 使用功能点初始化
    // @param wide_seed_search 个人修改（2026-10-01）：为 true 时先对种子做"粗分辨率 + 小范围 yaw 搜索"
    //        再进入细分辨率配准 —— 专用于【手动点选 / 外部给定】的初始位姿，用于吸收点击延迟
    //        与朝向误差；功能点(start/recover)等普通初始化传 false，行为与修改前完全一致。
    bool InitWithFP(CloudPtr input, const SE3& fp_pose, bool wide_seed_search = false);

    // 更新全局地图
    bool UpdateGlobalMap();

    // @brief 定位是否已经成功初始化
    bool LocInited();

    // @brief 激光定位重置接口
    void ResetLastPose(const SE3& last_pose);

    /**
     * @brief 激光地图匹配
     * @param pose       预测位姿
     * @param confidence    得分
     * @param input 输入点云
     * @param output 输出点云
     * @param use_rough_res 是否使用粗分辨率
     * @return
     */
    bool Localize(SE3& pose, double& confidence, CloudPtr input, CloudPtr output, bool use_rough_res = false);

    // 设置UI
    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) { ui_ = ui; }

    // 设置init pose
    void SetInitialPose(SE3 init_pose);

    // 获取定位结果
    LocalizationResult GetLocalizationResult() {
        UL lock(result_mutex_);
        return localization_result_;
    }

    void Finish();

    // 激光定位是否认为LO有效
    bool LidarLocThinkLOReliable() { return lo_reliable_; }

   private:
    // 内部函数  ==========================================================================
    /**
     * 对点云进行配准
     * @param input
     */
    void Align(const CloudPtr& input);

    /**
     * 寻找当前帧对应的LO相对位姿
     * @param timestamp
     * @return
     */
    bool AssignLOPose(double timestamp);

    /**
     * 寻找当前帧对应的DR相对位姿
     * @param timestamp
     * @return
     */
    bool AssignDRPose(double timestamp);

    /**
     * 检查车辆是否静止
     * @param timestamp
     * @return
     */
    bool CheckStatic(double timestamp);

    /**
     * 更新自身状态
     * @param input
     */
    void UpdateState(const CloudPtr& input);

    /**
     * 更新地图
     */
    void UpdateMapThread();

    /**
     * 使用网格搜索best yaw
     */
    bool YawSearch(SE3& pose, double& confidence, CloudPtr input, CloudPtr output);

    /**
     * 个人修改（2026-10-01，手动初始位姿配套）：以 seed 为中心，用【粗分辨率】NDT 在
     * ±options_.manual_init_yaw_search_deg_ 范围内搜索最优 yaw，返回分值最高的位姿。
     *   粗分辨率(5m)代价函数更平滑、收敛半径更大 ⇒ 可吸收手动点选的米级位置偏差与较大朝向偏差，
     *   之后再由细分辨率 NDT 精确收敛（分值门槛 min_init_confidence_ 仍由 Localize() 把关）。
     *   @return 是否至少成功算出一个候选（粗分辨率 NDT 未就绪时返回 false，调用方退回原种子）
     */
    bool SearchSeedRough(CloudPtr input, const SE3& seed, SE3& best_pose, double& best_score, CloudPtr out);

    bool CheckLidarOdomValid(const SE3& current_pose_esti, double& delta_posi);

    // 个人修改（方案C）：在 Align() 里对最终位姿施加"高度约束"（z 拉回 map_height_，见 Options 注释）
    void ApplyHeightConstraint(SE3& pose, const char* tag);

    // 个人修改（方案C）：地图地面健康度自检：对起点附近的地面点做平面拟合，
    //   给出地面倾角/高度并打日志（倾角过大 ⇒ 地图是倾斜的 ⇒ 定位 z 必然漂移，需要重建地图）
    void CheckMapLevel();

    // 成员变量  ==========================================================================
    Options options_;

    std::mutex match_mutex_;  // 锁定pcl_ndt指针

    using NDTType = pclomp::NormalDistributionsTransform<PointType, PointType>;
    NDTType::Ptr pcl_ndt_ = nullptr;
    NDTType::Ptr pcl_ndt_rough_ = nullptr;  // 粗分辨率

    using ICPType = pcl::IterativeClosestPoint<PointType, PointType>;
    ICPType::Ptr pcl_icp_ = nullptr;

    CloudPtr current_scan_ = nullptr;                   // 当前扫描
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;  // ui
    SE3 last_loc_pose_;

    std::mutex initial_pose_mutex_;  // 初始定位锁
    bool initial_pose_set_ = false;  // 定位是否被手动设置
    SE3 initial_pose_;               // 手动设置的初始位姿
    bool loc_inited_ = false;        // 定位是否初始化成功

    double current_timestamp_ = 0;  // 本次输入的时间戳
    double last_timestamp_ = 0;     // 上次输入的时间戳

    bool last_lo_pose_set_ = false;
    bool current_lo_pose_set_ = false;
    SE3 last_lo_pose_;     // 上一次相对位置，相对位置来自LO
    SE3 current_lo_pose_;  // 本次的LO相对位置

    bool last_dr_pose_set_ = false;
    bool current_dr_pose_set_ = false;
    SE3 last_dr_pose_;     // 上一次相对位置，相对位置来自DR
    SE3 current_dr_pose_;  // 本次的DR相对位置
    bool parking_ = false;

    double try_other_guess_trans_th_ = 0.3;               // 在初始估计相差多少时，尝试其他的解
    double try_other_guess_rot_th_ = 0.5 * M_PI / 180.0;  // 在初始估计相差多少时，尝试其他的解
    // double low_vel_th_ = 1.0;                             // 低速阈值m/s
    // double update_cache_dis_ = 0;                         // 动态图层的更新缓冲距离

    std::deque<double> ave_scores_;  // 近期匹配的分值情况

    Vec3d current_vel_b_ = Vec3d::Zero();  // 本次的车体系下的速度
    Vec3d current_vel_ = Vec3d::Zero();    // 本次dr的速度

    std::deque<TimedPose> lidar_loc_pose_queue_;  // lidar odom pose 轨迹

    bool lo_reliable_ = true;
    int lo_reliable_cnt_ = 0;

    SE3 last_abs_pose_;            // 上一次绝对定位，绝对定位来自于地图匹配得到的位姿
    TimedPose last_dyn_upd_pose_;  // 上次更新动态图层时使用的位姿
    SE3 current_abs_pose_;         // 本次的绝对定位
    bool last_abs_pose_set_ = false;
    double current_score_ = 1e5;  // 设一个大分值，若定位一开始就匹配失败，则可以直接用GPS重置
    int match_fail_count_ = 0;
    int static_count_ = 0;

    int rtk_reset_cnt_ = 0;  // RTK重置计数

    std::mutex result_mutex_;
    LocalizationResult localization_result_;  // 输出结果

    // 相对运动观测队列
    std::mutex lo_pose_mutex_;
    std::deque<NavState> lo_pose_queue_;

    std::mutex dr_pose_mutex_;
    std::deque<NavState> dr_pose_queue_;

    // 功能点初始化的记录
    std::vector<SE3> fp_init_fail_pose_vec_;
    double fp_last_tried_time_ = 0;

    bool update_map_quit_ = false;
    std::thread update_map_thread_;            // 地图更新
    std::shared_ptr<TiledMap> map_ = nullptr;  // 地图
    double map_height_ = 0;
    bool map_height_valid_ = false;  // 个人修改（方案C）：map_height_ 是否已有有效值（首帧功能点的高度）
    Vec4d map_ground_plane_ = Vec4d(0, 0, 1, 0);  // 个人修改（方案C）：自检出的地图地面平面 [a,b,c,d]
    double map_ground_tilt_deg_ = 0.0;            // 个人修改（方案C）：地面相对水平面倾角(度)

    bool has_set_pose_ = false;  // 外部set_pose标志位，若存在则本次动态图层不落盘

    std::ofstream recover_pose_out_;
};

}  // namespace lightning::loc
