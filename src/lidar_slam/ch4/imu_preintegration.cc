#include "lidar_slam/ch4/imu_preintegration.hh"

namespace slam_learn {
IMUPreintegration::IMUPreintegration(Options options) {
    bg_ = options.init_bg_;
    ba_ = options.init_ba_;
    const float ng2 = options.noise_gyro_ * options.noise_gyro_;
    const float na2 = options.noise_acce_ * options.noise_acce_;
    noise_gyro_acc_.diagonal() << ng2, ng2, ng2, na2, na2, na2;
}

/**
 * 插入新的IMU数据
 * @param imu   imu 读数
 * @param dt    时间差
 */
void IMUPreintegration::Integrate(const IMU &imu, double dt) {
    // 去掉零偏
    Vec3d gyro = imu.gyro_ - bg_;
    Vec3d acc = imu.acce_ - ba_;
    // 计算dp,dv (4.7)
    dp_ = dp_ + dv_ * dt + 0.5f * dR_.matrix() * acc * dt * dt;
    dv_ = dv_ + dR_ * acc * dt;
    // 先不更新dR,运动方程的jacobian矩阵
    // dp,dv,dr对\delta{theta, v, p}对jacobian矩阵
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    // dp,dv,dr对\delta{ba,bg}对jacobian矩阵
    Eigen::Matrix<double, 9, 6> B;
    B.setIdentity();

    Mat3d acc_hat = SO3::hat(acc);
    double dt2 = dt * dt2;
    A.block<3, 3>(3, 0) = -dR_.matrix() * dt * acc_hat;
    A.block<3, 3>(6, 0) = -0.5f * dR_.matrix() * acc_hat * dt2;
    A.block<3, 3>(6, 3) = dt * Mat3d::Identity();

    B.block<3, 3>(3, 3) = dR_.matrix() * dt;
    B.block<3, 3>(6, 3) = 0.5f * dR_.matrix() * dt2;

    // 更新对零偏的雅可比矩阵
    dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5f * dR_.matrix() * dt2;
    dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5f * dR_.matrix() * dt2 * acc_hat * dR_dbg_;
    dV_dba_ = dV_dba_ - dR_.matrix() * dt;
    dV_dbg_ = dV_dbg_ - dR_.matrix() * dt * acc_hat * dR_dbg_;

    // 更新dR
    Vec3d omega = gyro * dt;
    Mat3d rightJ = SO3::jr(omega);
    SO3 deltaR = SO3::exp(omega);
    dR_ = dR_ * deltaR;
    A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
    B.block<3, 3>(0, 0) = rightJ * dt;

    cov_ = A * cov_ * A.transpose() + B * noise_gyro_acc_ * B.transpose();
    dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt;

    dt_ += dt;
}

/**
 * 从某个起始点开始预测积分之后的状态
 * @param start 起始时时刻状态
 * @return  预测的状态
 */
NavStated IMUPreintegration::Predict(const NavStated &start, const Vec3d &grav) const {
    SO3 Rj = start.R_ * dR_;
    Vec3d Vj = start.R_ * dv_ + start.v_ + grav * dt_;
    Vec3d Pj = start.p_ + start.R_ * dp_ + start.v_ * dt_ + 0.5f * grav * dt_ * dt_;

    auto state = NavStated(start.timestamped_ + dt_, Rj, Pj, Vj);
    state.bg_ = bg_;
    state.ba_ = ba_;
    return state;
}

/// 获取修正之后的观测量，bias可以与预积分时期的不同，会有一阶修正
SO3 IMUPreintegration::GetDeltaRotation(const Vec3d &bg) {
    return dR_ * SO3::exp(dR_dbg_ * (bg - bg_));
}
Vec3d IMUPreintegration::GetDeltaVelocity(const Vec3d &bg, const Vec3d &ba) {
    return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
}
Vec3d IMUPreintegration::GetDeltaPosition(const Vec3d &bg, const Vec3d &ba) {
    return dp_ = dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
}
}  // namespace slam_learn