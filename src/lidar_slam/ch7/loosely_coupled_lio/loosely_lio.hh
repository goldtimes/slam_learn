#pragma once
/**
 * 7.5 节实现的松耦合LIO程序
 * 使用第3章的ekf, 7.3节的增量NDT里程计来实现
 * 事先从YAML中读取必要参数
 *
 * 由于是第一个有IMU和雷达的程序，框架变动较大
 * 后续紧耦合LIO将继续使用本框架
 */
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

#include "lidar_slam/ch3/eskf.hpp"
#include "lidar_slam/ch3/static_imu_init.hh"
#include "lidar_slam/ch7/incremental_ndt_lo.hh"
#include "lidar_slam/ch7/loosely_coupled_lio/cloud_convert.hh"
#include "lidar_slam/ch7/loosely_coupled_lio/measure_sync.hh"

#include "common/nav_state.hh"
#include "tools/ui/pangolin_window.hh"

namespace slam_learn::loosely {
using namespace ndt_inc;
using namespace imu_init;
using namespace inc_ndt_lo;
using namespace eskf;
class LooselyLIO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    struct Options {
        Options() {
        }
        bool save_motion_undistortion_pcd_ = false;  // 是否保存去畸变前后的点云
        bool with_ui_ = true;                        // 是否带着UI
    };

   public:
    LooselyLIO(Options options);
    ~LooselyLIO() = default;

    /// 从配置文件初始化
    bool Init(const std::string &config_yaml);

    /// 点云回调函数
    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);

    /// IMU回调函数
    void IMUCallBack(IMUPtr msg_in);

    /// 结束程序，退出UI
    void Finish();

   private:
    /// 处理同步之后的IMU和雷达数据
    void ProcessMeasurements(const MeasureGroup &meas);

    bool LoadFromYAML(const std::string &yaml);

    /// 尝试让IMU初始化
    void TryInitIMU();

    /// 利用IMU预测状态信息
    /// 这段时间的预测数据会放入imu_states_里
    void Predict();

    /// 对measures_中的点云去畸变
    void Undistort();

    /// 执行一次配准和观测
    void Align();

   private:
    /// modules
    std::shared_ptr<MessageSync> sync_ = nullptr;  // 消息同步器
    StaticIMUInit imu_init_;                       // IMU静止初始化
    std::shared_ptr<IncrementalNDTLO> inc_ndt_lo_ = nullptr;

    /// point clouds data
    FullCloudPtr scan_undistort_{new FullPointCloudType()};  // scan after undistortion
    SE3 pose_of_lo_;

    Options options_;

    // flags
    bool imu_need_init_ = true;   // 是否需要估计IMU初始零偏
    bool flg_first_scan_ = true;  // 是否第一个雷达
    int frame_num_ = 0;           // 帧数计数

    // EKF data
    MeasureGroup measures_;              // 同步之后的IMU和点云
    std::vector<NavStated> imu_states_;  // ESKF预测期间的状态
    ESKFD eskf_;                         // ESKF
    SE3 TIL_;                            // Lidar与IMU之间外参

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};
}  // namespace slam_learn::loosely
