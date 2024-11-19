#include "v_slam/ch5/code_ba_schur/backend/edge_imu.hh"
#include <iostream>
#include "v_slam/ch5/code_ba_schur/backend/vertex_motion.hh"
#include "v_slam/ch5/code_ba_schur/backend/vertex_pose.hh"

namespace slam_learn::backend {

using Sophus::SO3d;
Eigen::Vector3d EdgeImu::gravity_ = Eigen::Vector3d(0, 0, 9.8);

void EdgeImu::ComputeResidual() {
    // v1--v2---
    // p,q,  v,bg,ba
    Eigen::VectorXd param_0 = verticies_[0]->Parameters();
    Eigen::Quaterniond qi(param_0[6], param_0[3], param_0[4], param_0[5]);
    Vec3 pi = param_0.head<3>();
    SO3d ri(qi);
    SO3d ri_inv = ri.inverse();

    Eigen::VectorXd param_1 = verticies_[1]->Parameters();
    Vec3 vi = param_1.head<3>();
    Vec3 bai = param_1.segment(3, 3);
    Vec3 bgi = param_1.tail<3>();

    Eigen::VectorXd param_2 = verticies_[2]->Parameters();
    Eigen::Quaterniond qj(param_2[6], param_2[3], param_2[4], param_2[5]);
    Vec3 pj = param_2.head<3>();

    Eigen::VectorXd param_3 = verticies_[3]->Parameters();
    Vec3 vj = param_3.head<3>();
    Vec3 baj = param_3.segment(3, 3);
    Vec3 bgj = param_3.tail<3>();
    SO3d rj(qj);

    double dt = pre_integration_->GetSumDt();
    double dt2 = dt * dt;
    SO3d dr;
    Vec3 dv;
    Vec3 dp;
    pre_integration_->GetDeltaRVP(dr, dv, dp);

    SO3d res_r = dr.inverse() * ri_inv * rj;
    // 残差是简单的
    residual_.block<3, 1>(0, 0) = Sophus::SO3d::log(res_r);
    residual_.block<3, 1>(3, 0) = ri_inv * (vj - vi - gravity_ * dt) - dv;
    residual_.block<3, 1>(6, 0) = ri_inv * (pj - pi - vi * dt - 0.5 * gravity_ * dt2) - dp;
    residual_.block<3, 1>(9, 0) = baj - bai;
    residual_.block<3, 1>(12, 0) = bgj - bgi;
}

void EdgeImu::ComputeJacobians() {
    Eigen::VectorXd param_0 = verticies_[0]->Parameters();
    Eigen::Quaterniond qi(param_0[6], param_0[3], param_0[4], param_0[5]);
    Vec3 pi = param_0.head<3>();
    SO3d ri(qi);
    SO3d ri_inv = ri.inverse();

    Eigen::VectorXd param_1 = verticies_[1]->Parameters();
    Vec3 vi = param_1.head<3>();
    Vec3 bai = param_1.segment(3, 3);
    Vec3 bgi = param_1.tail<3>();

    Eigen::VectorXd param_2 = verticies_[2]->Parameters();
    Eigen::Quaterniond qj(param_2[6], param_2[3], param_2[4], param_2[5]);
    Vec3 pj = param_2.head<3>();

    Eigen::VectorXd param_3 = verticies_[3]->Parameters();
    Vec3 vj = param_3.head<3>();
    Vec3 baj = param_3.segment(3, 3);
    Vec3 bgj = param_3.tail<3>();
    SO3d rj(qj);
    SO3d rj_inv = rj.inverse();

    double dt = pre_integration_->GetSumDt();
    double dt2 = dt * dt;
    SO3d dr;
    Vec3 dv;
    Vec3 dp;
    pre_integration_->GetDeltaRVP(dr, dv, dp);
    SO3d res_r = dr.inverse() * ri_inv * rj;

    const Mat33 i3 = Mat33::Identity();

    // w.r.t. pose i
    Eigen::Matrix<double, 15, 7> jacobian_pose_i = Eigen::Matrix<double, 15, 7>::Zero();
    // dr/dp = 0
    // dr/dr
    jacobian_pose_i.block<3, 3>(0, 3) = -SO3d::jr_inv(res_r.log()) * (rj_inv * ri).matrix();
    // dv/dr
    jacobian_pose_i.block<3, 3>(3, 3) = SO3d::hat(ri_inv * (-gravity_ * dt + vj - vi));
    // dp/dp
    jacobian_pose_i.block<3, 3>(6, 0) = -i3;
    // dp/dr
    jacobian_pose_i.block<3, 3>(6, 3) = SO3d::hat(ri_inv * (pj - pi - vi * dt - 0.5 * gravity_ * dt2));

    // w.r.t. speed and bias in i
    Eigen::Matrix<double, 15, 9> jacobian_speedbias_i = Eigen::Matrix<double, 15, 9>::Zero();
    // dr/dv =0 ,dr/dba=0
    jacobian_speedbias_i.block<3, 3>(0, 6) = -SO3d::jr_inv(res_r.log()) * dr.inverse().matrix() * dr_dbg_;
    // dv/dv
    jacobian_speedbias_i.block<3, 3>(3, 0) = -ri_inv.matrix();
    // dv/dba
    jacobian_speedbias_i.block<3, 3>(3, 3) = -dv_dba_;
    // dv/dbg
    jacobian_speedbias_i.block<3, 3>(3, 6) = -dv_dbg_;
    // dp/dv
    jacobian_speedbias_i.block<3, 3>(6, 0) = -ri_inv.matrix() * dt;
    // dp/dba, dp/dbg
    jacobian_speedbias_i.block<3, 3>(6, 3) = -dp_dba_;
    jacobian_speedbias_i.block<3, 3>(6, 6) = -dp_dbg_;
    // dba/dba
    jacobian_speedbias_i.block<3, 3>(9, 3) = -i3;
    // dbg/dbg
    jacobian_speedbias_i.block<3, 3>(12, 6) = -i3;

    // w.r.t. pose j
    Eigen::Matrix<double, 15, 7> jacobian_pose_j = Eigen::Matrix<double, 15, 7>::Zero();
    jacobian_pose_j.block<3, 3>(0, 3) = SO3d::jr_inv(res_r.log());
    jacobian_pose_j.block<3, 3>(6, 0) = (ri_inv * rj).matrix();

    // w.r.t. speed and bias j
    Eigen::Matrix<double, 15, 9> jacobian_speedbias_j = Eigen::Matrix<double, 15, 9>::Zero();
    // dv/dv
    jacobian_speedbias_j.block<3, 3>(3, 0) = ri_inv.matrix();
    // dba/dba
    jacobian_speedbias_j.block<3, 3>(9, 3) = i3;
    jacobian_speedbias_j.block<3, 3>(12, 6) = i3;

    jacobians_[0] = jacobian_pose_i;
    jacobians_[1] = jacobian_speedbias_i;
    jacobians_[2] = jacobian_pose_j;
    jacobians_[3] = jacobian_speedbias_j;
}

}  // namespace slam_learn::backend
