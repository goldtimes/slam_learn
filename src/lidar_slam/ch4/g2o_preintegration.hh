#pragma once
// binary是连接两个顶点的边
#include <g2o/core/base_binary_edge.h>
// base_multi_edge连接多个顶点的边
#include <g2o/core/base_multi_edge.h>
// 连接一个顶点的边
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>

#include <g2o/core/robust_kernel_impl.h>
#include <glog/logging.h>

#include <memory>
#include "common/eigen_types.hh"
#include "common/g2o_types.hh"
#include "lidar_slam/ch4/imu_preintegration.hh"

namespace slam_learn {
/**
 * @brief 预积分的边，连接6个顶点，上一时刻T，v, bg, ba 此刻的T,v
 * 9 为观测量的维度，也就是预积分的观测量 \deltaR, \deltaV \deltaP,
 */
class EdgeInertial : public g2o::BaseMultiEdge<9, Vec9d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInertial() = default;
    EdgeInertial(std::shared_ptr<IMUPreintegration> pre_inte, const Vec3d& gravity, double weight = 1.0);

    bool read(std::istream& is) override {
        return false;
    }
    bool write(std::ostream& os) const override {
        return false;
    }

    void computeError() override;
    void linearizeOplus() override;
    // 将T拆为r,t之后总共8个顶点，8*3的24维
    Eigen::Matrix<double, 24, 24> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        // 残差为9维，对上一时刻的r,t的jacobian
        J.block<9, 6>(0, 0) = _jacobianOplus[0];
        // 残差为9维，对上一时刻的v
        J.block<9, 3>(0, 6) = _jacobianOplus[1];
        // 残差为9维，对上一时刻的bg
        J.block<9, 3>(0, 9) = _jacobianOplus[2];
        // 残差为9维，对上一时刻的ba
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        // 残差为9维，对此刻的r,t的jacobian
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        // 残差为9维，对此刻的v
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

   private:
    const double dt_;
    std::shared_ptr<IMUPreintegration> preint_ = nullptr;
    Vec3d grav_;
};
}  // namespace slam_learn