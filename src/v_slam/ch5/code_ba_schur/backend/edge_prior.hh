#pragma once
#include <memory>
#include <string>
#include "v_slam/ch5/code_ba_schur/backend/edge.hh"

#include <Eigen/Dense>
namespace slam_learn::backend {
class EdgeSE3Prior : public Edge {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Vec3 = Eigen::Vector3d;
    using Qd = Eigen::Quaterniond;
    EdgeSE3Prior(const Vec3 &p, const Qd &q) : Edge(6, 1, std::vector<std::string>{"VertexPose"}), Pp_(p), Qp_(q) {
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "EdgeSE3Prior";
    }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

   private:
    Vec3 Pp_;  // pose prior
    Qd Qp_;    // Rotation prior
};
}  // namespace slam_learn::backend