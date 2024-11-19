#pragma once
#include <memory>
#include <string>

#include <Eigen/Dense>
#include "v_slam/ch5/code_ba_schur/backend/edge.hh"

namespace slam_learn::backend {
/**
 * 此边是视觉重投影误差，此边为三元边，与之相连的顶点有：
 * 路标点的逆深度InveseDepth、第一次观测到该路标点的source Camera的位姿T_World_From_Body1，
 * 和观测到该路标点的mearsurement Camera位姿T_World_From_Body2。
 * 注意：verticies_顶点顺序必须为InveseDepth、T_World_From_Body1、T_World_From_Body2。
 */

class EdgeReprojection : public Edge {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Vec3 = Eigen::Vector3d;
    using Qd = Eigen::Quaterniond;
    using VecX = Eigen::VectorXd;
    using Mat33 = Eigen::Matrix<double, 3, 3>;
    EdgeReprojection(const Vec3 &pts_i, const Vec3 &pts_j)
        : Edge(2, 3, std::vector<std::string>{"VertexInverseDepth", "VertexPose", "VertexPose"}) {
        pts_i_ = pts_i;
        pts_j_ = pts_j;
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "EdgeReprojection";
    }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

    void SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Vec3 &tic_);

   private:
    // Translation imu from camera
    Qd qic;
    Vec3 tic;

    // measurements
    Vec3 pts_i_, pts_j_;
};

/**
 * 此边是视觉重投影误差，此边为二元边，与之相连的顶点有：
 * 路标点的世界坐标系XYZ、观测到该路标点的 Camera 的位姿T_World_From_Body1
 * 注意：verticies_顶点顺序必须为 XYZ、T_World_From_Body1。
 */
class EdgeReprojectionXYZ : public Edge {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Vec3 = Eigen::Vector3d;
    using Qd = Eigen::Quaterniond;
    using VecX = Eigen::VectorXd;
    using Mat33 = Eigen::Matrix<double, 3, 3>;
    EdgeReprojectionXYZ(const Vec3 &pts_i) : Edge(2, 2, std::vector<std::string>{"VertexXYZ", "VertexPose"}) {
        obs_ = pts_i;
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "EdgeReprojectionXYZ";
    }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

    void SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Vec3 &tic_);

   private:
    // Translation imu from camera
    Qd qic;
    Vec3 tic;

    // measurements
    Vec3 obs_;
};

}  // namespace slam_learn::backend