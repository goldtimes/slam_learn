#include "lidar_slam/ch8/lio-preinteg/lio_preinteg.hh"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <execution>

#include "common/g2o_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/sensors/point_type.hh"
#include "common/timer/timer.hh"
#include "lidar_slam/ch4/g2o_preintegration.hh"

namespace slam_learn::lio_preinteg {
LioPreinteg::LioPreinteg(Options options) {
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;  // 本节数据不需要轮速计
    imu_init_ = StaticIMUInit(imu_init_options);

    double bg_rw2 = 1.0 / (options_.bias_gyro_var_ * options_.bias_gyro_var_);
    options_.bg_rw_info_.diagonal() << bg_rw2, bg_rw2, bg_rw2;
    double ba_rw2 = 1.0 / (options_.bias_acce_var_ * options_.bias_acce_var_);
    options_.ba_rw_info_.diagonal() << ba_rw2, ba_rw2, ba_rw2;

    double gp2 = options_.ndt_pos_noise_ * options_.ndt_pos_noise_;
    double ga2 = options_.ndt_ang_noise_ * options_.ndt_ang_noise_;

    options_.ndt_info_.diagonal() << 1.0 / ga2, 1.0 / ga2, 1.0 / ga2, 1.0 / gp2, 1.0 / gp2, 1.0 / gp2;

    options_.ndt_options_.nearby_type_ = IncNdt3d::NearbyType::CENTER;
    ndt_ = IncNdt3d(options_.ndt_options_);
}
/// init without ros
bool LioPreinteg::Init(const std::string& config_yaml) {
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

bool LioPreinteg::LoadFromYAML(const std::string& yaml_file) {
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

void LioPreinteg::ProcessMeasurements(const MeasureGroup& meas) {
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

void LioPreinteg::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        // 噪声由初始化器估计
        options_.preinteg_options_.noise_gyro_ = sqrt(imu_init_.GetCovGyro()[0]);
        options_.preinteg_options_.noise_acce_ = sqrt(imu_init_.GetCovAcce()[0]);
        options_.preinteg_options_.init_ba_ = imu_init_.GetInitBa();
        options_.preinteg_options_.init_bg_ = imu_init_.GetInitBg();

        preinteg_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);
        imu_need_init_ = false;

        current_nav_state_.v_.setZero();
        current_nav_state_.bg_ = imu_init_.GetInitBg();
        current_nav_state_.ba_ = imu_init_.GetInitBa();
        current_nav_state_.timestamped_ = measures_.imu_.back()->timestamp_;

        last_nav_state_ = current_nav_state_;
        last_imu_ = measures_.imu_.back();

        LOG(INFO) << "IMU初始化成功";
    }
}

void LioPreinteg::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(last_nav_state_);

    /// 对IMU状态进行预测
    for (auto& imu : measures_.imu_) {
        if (last_imu_ != nullptr) {
            preinteg_->Integrate(*imu, imu->timestamp_ - last_imu_->timestamp_);
        }

        last_imu_ = imu;
        imu_states_.emplace_back(preinteg_->Predict(last_nav_state_, imu_init_.GetGravity()));
    }
}

void LioPreinteg::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_end_state = imu_states_.back();
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

void LioPreinteg::Align() {
    FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix().cast<float>());
    scan_undistort_ = scan_undistort_trans;

    current_scan_ = lidar_utils::ConvertToCloud<FullPointType>(scan_undistort_);

    // voxel 之
    pcl::VoxelGrid<PointXYZI> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan_);

    CloudPtr current_scan_filter(new PointCloudXYZI);
    voxel.filter(*current_scan_filter);

    /// the first scan
    if (flg_first_scan_) {
        ndt_.AddCloud(current_scan_);
        // 第一帧雷达数据来了，于是我们清空了这段时间的预积分值
        preinteg_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);
        flg_first_scan_ = false;
        return;
    }
    // 后续的scan，使用NDT配合pose进行更新
    LOG(INFO) << "=== frame " << frame_num_;
    ndt_.SetSource(current_scan_filter);
    // 这里已经产生了预积分的量
    // 得到预测的位姿
    current_nav_state_ = preinteg_->Predict(last_nav_state_, imu_init_.GetGravity());
    ndt_pose_ = current_nav_state_.GetSE3();
    // 配准
    ndt_.AlignNdt(ndt_pose_);
    // 图优化
    Optimize();
    // 若运动了一定范围，则把点云放入地图中
    SE3 current_pose = current_nav_state_.GetSE3();
    SE3 delta_pose = last_ndt_pose_.inverse() * current_pose;

    if (delta_pose.translation().norm() > 0.3 || delta_pose.so3().log().norm() > 5.0 * math::kDEG2RAD) {
        // 将地图合入NDT中
        CloudPtr current_scan_world(new PointCloudXYZI);
        pcl::transformPointCloud(*current_scan_filter, *current_scan_world, current_pose.matrix());
        ndt_.AddCloud(current_scan_world);
        last_ndt_pose_ = current_pose;
    }

    // 放入UI
    if (ui_) {
        ui_->UpdateScan(current_scan_, current_nav_state_.GetSE3());  // 转成Lidar Pose传给UI
        ui_->UpdateNavState(current_nav_state_);
    }

    frame_num_++;
}

// 要想着构建的图，然后编写代码
void LioPreinteg::Optimize() {
    // 调用g2o求解优化问题
    // 上一个state到本时刻state的预积分因子，本时刻的NDT因子
    LOG(INFO) << " === optimizing frame " << frame_num_ << " === "
              << ", dt: " << preinteg_->dt_;
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    // 求解器
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    // 优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 添加上一时刻的顶点4个(pose,v, bg, ba)
    auto v0_pose = new VertexPose();
    v0_pose->setId(0);
    v0_pose->setEstimate(last_nav_state_.GetSE3());
    optimizer.addVertex(v0_pose);

    auto v0_velocity = new VertexVelocity();
    v0_velocity->setId(1);
    v0_velocity->setEstimate(last_nav_state_.v_);
    optimizer.addVertex(v0_velocity);

    auto v0_bg = new VertexGyroBias();
    v0_bg->setId(2);
    v0_bg->setEstimate(last_nav_state_.bg_);

    auto v0_ba = new VertexAcceBias();
    v0_bg->setId(3);
    v0_bg->setEstimate(last_nav_state_.ba_);

    // 添加此刻的顶点

    auto v1_pose = new VertexPose();
    v1_pose->setId(4);
    v1_pose->setEstimate(current_nav_state_.GetSE3());
    optimizer.addVertex(v1_pose);

    auto v1_velocity = new VertexVelocity();
    v1_velocity->setId(5);
    v1_velocity->setEstimate(current_nav_state_.v_);
    optimizer.addVertex(v1_velocity);

    auto v1_bg = new VertexGyroBias();
    v1_bg->setId(6);
    v1_bg->setEstimate(current_nav_state_.bg_);

    auto v1_ba = new VertexAcceBias();
    v1_ba->setId(7);
    v1_ba->setEstimate(current_nav_state_.ba_);
    // 构建预积分的边
    auto edge_inertial = new EdgeInertial(preinteg_, imu_init_.GetGravity());
    edge_inertial->setVertex(0, v0_pose);
    edge_inertial->setVertex(1, v0_velocity);
    edge_inertial->setVertex(2, v0_bg);
    edge_inertial->setVertex(3, v0_ba);
    edge_inertial->setVertex(4, v1_pose);
    edge_inertial->setVertex(5, v1_velocity);
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(200.0);
    edge_inertial->setRobustKernel(rk);
    optimizer.addEdge(edge_inertial);

    // 零偏随机游走
    auto* edge_gyro_rw = new EdgeGyroRW();
    edge_gyro_rw->setVertex(0, v0_bg);
    edge_gyro_rw->setVertex(1, v1_bg);
    edge_gyro_rw->setInformation(options_.bg_rw_info_);
    optimizer.addEdge(edge_gyro_rw);

    auto* edge_acc_rw = new EdgeAcceRW();
    edge_acc_rw->setVertex(0, v0_ba);
    edge_acc_rw->setVertex(1, v1_ba);
    edge_acc_rw->setInformation(options_.ba_rw_info_);
    optimizer.addEdge(edge_acc_rw);

    // 上一帧pose, vel, bg, ba的先验
    auto* edge_prior = new EdgePriorPoseNavState(last_nav_state_, prior_info_);
    edge_prior->setVertex(0, v0_pose);
    edge_prior->setVertex(1, v0_velocity);
    edge_prior->setVertex(2, v0_bg);
    edge_prior->setVertex(3, v0_ba);
    optimizer.addEdge(edge_prior);

    /// 使用NDT的pose进行观测
    auto* edge_ndt = new EdgeGNSS(v1_pose, ndt_pose_);
    edge_ndt->setInformation(options_.ndt_info_);
    optimizer.addEdge(edge_ndt);
    if (options_.verbose_) {
        LOG(INFO) << "last: " << last_nav_state_;
        LOG(INFO) << "pred: " << current_nav_state_;
        LOG(INFO) << "NDT: " << ndt_pose_.translation().transpose() << ","
                  << ndt_pose_.so3().unit_quaternion().coeffs().transpose();
    }

    v0_bg->setFixed(true);
    v0_ba->setFixed(true);

    // go
    optimizer.setVerbose(options_.verbose_);
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // get results
    last_nav_state_.R_ = v0_pose->estimate().so3();
    last_nav_state_.p_ = v0_pose->estimate().translation();
    last_nav_state_.v_ = v0_velocity->estimate();
    last_nav_state_.bg_ = v0_bg->estimate();
    last_nav_state_.ba_ = v0_ba->estimate();

    current_nav_state_.R_ = v1_pose->estimate().so3();
    current_nav_state_.p_ = v1_pose->estimate().translation();
    current_nav_state_.v_ = v1_velocity->estimate();
    current_nav_state_.bg_ = v1_bg->estimate();
    current_nav_state_.ba_ = v1_ba->estimate();

    if (options_.verbose_) {
        LOG(INFO) << "last changed to: " << last_nav_state_;
        LOG(INFO) << "curr changed to: " << current_nav_state_;
        LOG(INFO) << "preinteg chi2: " << edge_inertial->chi2() << ", err: " << edge_inertial->error().transpose();
        LOG(INFO) << "prior chi2: " << edge_prior->chi2() << ", err: " << edge_prior->error().transpose();
        LOG(INFO) << "ndt: " << edge_ndt->chi2() << "/" << edge_ndt->error().transpose();
    }
}

/// 点云回调函数
void LioPreinteg::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sync_->ProcessCloud(msg);
}
void LioPreinteg::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    sync_->ProcessCloud(msg);
}

/// IMU回调函数
void LioPreinteg::IMUCallBack(IMUPtr msg_in) {
    sync_->ProcessIMU(msg_in);
}

void LioPreinteg::Finish() {
    if (ui_) {
        ui_->Quit();
    }
    LOG(INFO) << "finish done";
}
}  // namespace slam_learn::lio_preinteg