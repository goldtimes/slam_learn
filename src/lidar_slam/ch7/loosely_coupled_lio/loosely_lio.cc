#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <execution>
#include <memory>

#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/nav_state.hh"
#include "common/sensors/imu.hh"
#include "common/sensors/point_type.hh"
#include "common/timer/timer.hh"
#include "lidar_slam/ch3/eskf.hpp"
#include "lidar_slam/ch3/static_imu_init.hh"
#include "lidar_slam/ch7/incremental_ndt_lo.hh"
#include "lidar_slam/ch7/loosely_coupled_lio/loosely_lio.hh"
#include "lidar_slam/ch7/loosely_coupled_lio/measure_sync.hh"

namespace slam_learn::loosely {
LooselyLIO::LooselyLIO(Options options) {
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;
    imu_init_ = StaticIMUInit(imu_init_options);
}

/// 从配置文件初始化
bool LooselyLIO::Init(const std::string &config_yaml) {
    /// 初始化自身的参数
    if (!LoadFromYAML(config_yaml)) {
        return false;
    }
    // 增量ndt
    IncrementalNDTLO::Options indt_options;
    indt_options.display_realtime_cloud_ = false;
    inc_ndt_lo_ = std::make_shared<IncrementalNDTLO>(indt_options);

    /// 初始化UI
    if (options_.with_ui_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();
    }

    return true;
}

bool LooselyLIO::LoadFromYAML(const std::string &yaml_file) {
    sync_ = std::make_shared<loosely::MessageSync>([&](const MeasureGroup &group) { ProcessMeasurements(group); });
    sync_->Init(yaml_file);

    // lidar到imu外参

    /// 自身参数主要是雷达与IMU外参
    auto yaml = YAML::LoadFile(yaml_file);
    std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
    std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

    Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
    Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    return true;
}

/// 处理同步之后的IMU和雷达数据
void LooselyLIO::ProcessMeasurements(const MeasureGroup &meas) {
    LOG(INFO) << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_->size();
    measures_ = meas;
    if (imu_need_init_) {
        TryInitIMU();
        return;
    }
    // imu进行预测
    Predict();
    // 去畸变
    Undistort();
    // 配准
    Align();
}

/// 尝试让IMU初始化
void LooselyLIO::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }
    // 初始化成功
    if (imu_init_.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        ESKFD::Options eskf_options;
        eskf_options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        eskf_options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);

        eskf_.SetInitialConditions(eskf_options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;

        LOG(INFO) << "IMU初始化成功";
    }
}

/// 点云回调函数
void LooselyLIO::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sync_->ProcessCloud(msg);
}
void LooselyLIO::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    sync_->ProcessCloud(msg);
}

/// IMU回调函数
void LooselyLIO::IMUCallBack(IMUPtr msg_in) {
    sync_->ProcessIMU(msg_in);
}

/// 利用IMU预测状态信息
/// 这段时间的预测数据会放入imu_states_里
void LooselyLIO::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(eskf_.GetNominalState());
    for (const auto &imu : measures_.imu_) {
        eskf_.Predict(*imu);
        imu_states_.push_back(eskf_.GetNominalState());
    }
}

/// 对measures_中的点云去畸变
void LooselyLIO::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_end_state = eskf_.GetNominalState();
    // 取出最后时刻的姿态
    SE3 T_end = SE3(imu_end_state.R_, imu_end_state.p_);

    // 将所有点转换到最后时刻
    std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](FullPointType &pt) {
        SE3 Ti;
        NavStated match;
        math::PoseInterp<NavStated>(measures_.lidar_begin_time + pt.time * (double)1e-3, imu_states_,
                                    [](const NavStated &state) { return state.timestamped_; },
                                    [](const NavStated &state) { return state.GetSE3(); }, Ti, match);
        Vec3d pt_eigen = lidar_utils::ToVec3d(pt);
        // 去畸变后的点
        // Ti * TIL_ * pt_eigen 得到点在i时刻下的世界位置 * T_end.inverse()
        // T_LI * T_Iend_W * T_WIi * T_IL * p_L;
        Vec3d pt_undistort = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pt_eigen;
        pt.x = pt_undistort(0);
        pt.y = pt_undistort(1);
        pt.z = pt_undistort(2);
    });
    scan_undistort_ = cloud;
}

/// 执行一次配准和观测
void LooselyLIO::Align() {
    FullCloudPtr scan_undistort_body(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_body, TIL_.matrix());
    scan_undistort_ = scan_undistort_body;
    // filter
    auto current_scan = lidar_utils::ConvertToCloud(scan_undistort_);
    lidar_utils::VoxelGrid(current_scan, 0.5);
    // 处理首帧雷达
    /// 处理首帧雷达数据
    if (flg_first_scan_) {
        SE3 pose;
        inc_ndt_lo_->AddCloud(current_scan, pose);
        flg_first_scan_ = false;
        return;
    }
    /// 从EKF中获取预测pose，放入LO，获取LO位姿，最后合入EKF
    SE3 pose_predict = eskf_.GetNominalSE3();
    inc_ndt_lo_->AddCloud(current_scan, pose_predict, true);
    pose_of_lo_ = pose_predict;
    eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2);
    if (options_.with_ui_) {
        // 放入UI
        ui_->UpdateScan(current_scan, eskf_.GetNominalSE3());  // 转成Lidar Pose传给UI
        ui_->UpdateNavState(eskf_.GetNominalState());
    }
    frame_num_++;
}

/// 结束程序，退出UI
void LooselyLIO::Finish() {
    if (options_.with_ui_) {
        while (ui_->ShouldQuit() == false) {
            usleep(1e5);
        }

        ui_->Quit();
    }
    LOG(INFO) << "finish done";
}

}  // namespace slam_learn::loosely