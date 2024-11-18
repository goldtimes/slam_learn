#include "v_slam/ch5/code_ba_schur/backend/vertex_pose.hh"

#include "sophus/so3.hpp"

namespace slam_learn::backend {
void VertexPose::Plus(const Eigen::VectorXd& delta) {
    Eigen::VectorXd& parameters = Parameters();
    parameters.head<3>() += delta.head<3>();
    Eigen::Quaterniond q(parameters[6], parameters[3], parameters[4], parameters[5]);
    // 右乘更新
    q = q * Sophus::SO3d::exp(Eigen::Vector3d(delta[3], delta[4], delta[5])).unit_quaternion();
    q.normalize();
    parameters[3] = q.x();
    parameters[4] = q.y();
    parameters[5] = q.z();
    parameters[6] = q.w();
}
}  // namespace slam_learn::backend
