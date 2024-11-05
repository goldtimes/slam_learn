#include "lidar_slam/ch8/lio-iekf/lio_iekf.hh"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/sensors/point_type.hh"
namespace slam_learn::ieskf_lio {
LioIEKF::LioIEKF(Options options) {
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;  // 本节数据不需要轮速计
    imu_init_ = StaticIMUInit(imu_init_options);
}

/// init without ros
bool LioIEKF::Init(const std::string& config_yaml) {
    if (!LoadFromYAML(config_yaml)) {
        LOG(INFO) << "init failed.";
        return false;
    }

    if (options_.with_ui_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();
    }

    return true;
}

bool LioIEKF::LoadFromYAML(const std::string& yaml_file) {
    // get params from yaml
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup& m) { ProcessMeasurements(m); });
    sync_->Init(yaml_file);

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
void LioIEKF::ProcessMeasurements(const MeasureGroup& meas) {
    LOG(INFO) << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_->size();
    measures_ = meas;

    if (imu_need_init_) {
        // 初始化IMU系统
        TryInitIMU();
        return;
    }

    // 利用IMU数据进行状态预测
    Predict();

    // 对点云去畸变
    Undistort();

    // 配准
    Align();
}

/// 尝试让IMU初始化
void LioIEKF::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        IESKFD::Options options;
        // 噪声由初始化器估计
        options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
        ieskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;

        LOG(INFO) << "IMU初始化成功";
    }
}

/// 利用IMU预测状态信息
/// 这段时间的预测数据会放入imu_states_里
void LioIEKF::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(ieskf_.GetNominalState());

    /// 对IMU状态进行预测
    for (auto& imu : measures_.imu_) {
        ieskf_.Predict(*imu);
        imu_states_.emplace_back(ieskf_.GetNominalState());
    }
}

/// 对measures_中的点云去畸变
void LioIEKF::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_end_state = ieskf_.GetNominalState();
    // 取出最后时刻的姿态
    SE3 T_end = SE3(imu_end_state.R_, imu_end_state.p_);

    // 将所有点转换到最后时刻
    std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](FullPointType& pt) {
        SE3 Ti;
        NavStated match;
        math::PoseInterp<NavStated>(measures_.lidar_begin_time + pt.time * (double)1e-3, imu_states_,
                                    [](const NavStated& state) { return state.timestamped_; },
                                    [](const NavStated& state) { return state.GetSE3(); }, Ti, match);
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
void LioIEKF::Align() {
    FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix().cast<float>());
    scan_undistort_ = scan_undistort_trans;

    current_scan_ = lidar_utils::ConvertToCloud<FullPointType>(scan_undistort_);
    lidar_utils::VoxelGrid(current_scan_, 0.5);

    /// the first scan
    if (flg_first_scan_) {
        ndt_.AddCloud(current_scan_);
        flg_first_scan_ = false;

        return;
    }

    // 后续的scan，使用NDT配合pose进行更新
    LOG(INFO) << "=== frame " << frame_num_;

    ndt_.SetSource(current_scan_);
    // 更新
    ieskf_.UpdateUsingCustomObserve([this](const SE3& input_pose, Mat18d& HTVH, Vec18d& HTVr) {
        ndt_.ComputeResidualAndJacobians(input_pose, HTVH, HTVr);
    });
    auto current_nav_state = ieskf_.GetNominalState();

    // 若运动了一定范围，则把点云放入地图中
    SE3 current_pose = ieskf_.GetNominalSE3();
    SE3 delta_pose = last_pose_.inverse() * current_pose;

    if (delta_pose.translation().norm() > 1.0 || delta_pose.so3().log().norm() > 10.0 * math::kDEG2RAD) {
        // 将地图合入NDT中
        CloudPtr current_scan_world(new PointCloudXYZI);
        pcl::transformPointCloud(*current_scan_, *current_scan_world, current_pose.matrix());
        ndt_.AddCloud(current_scan_world);
        last_pose_ = current_pose;
    }

    // 放入UI
    if (ui_) {
        ui_->UpdateScan(current_scan_, current_nav_state.GetSE3());  // 转成Lidar Pose传给UI
        ui_->UpdateNavState(current_nav_state);
    }

    frame_num_++;
    return;
}

/// 点云回调函数
void LioIEKF::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sync_->ProcessCloud(msg);
}
void LioIEKF::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    sync_->ProcessCloud(msg);
}

/// IMU回调函数
void LioIEKF::IMUCallBack(IMUPtr msg_in) {
    sync_->ProcessIMU(msg_in);
}

/// 结束程序，退出UI
void LioIEKF::Finish() {
    if (ui_) {
        ui_->Quit();
    }
    LOG(INFO) << "finish done";
}

}  // namespace slam_learn::ieskf_lio