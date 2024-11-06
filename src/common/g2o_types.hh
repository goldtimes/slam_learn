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
#include "common/sensors/gnss.hh"
#include "common/sensors/imu.hh"
#include "lidar_slam/ch4/imu_preintegration.hh"

namespace slam_learn {
/**
 * 旋转在前的SO3+t类型pose，6自由度，存储时伪装为g2o::VertexSE3，供g2o_viewer查看
 * 这里只需要告诉顶点是如何更新的
 */
class VertexPose : public g2o::BaseVertex<6, SE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() {
    }
    // read
    bool read(std::istream& is) override {
        double data[7];
        for (int i = 0; i < 7; i++) {
            is >> data[i];
        }
        setEstimate(SE3(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2])));
        return true;
    }

    bool write(std::ostream& os) const override {
        os << "VERTEX_SE3:QUAT ";
        os << id() << " ";
        Quatd q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
        return true;
    }
    // write

    virtual void setToOriginImpl() {
    }
    // 更新
    virtual void oplusImpl(const double* update_) {
        // 旋转部分的更新
        // Eigen::Map<const Vec3d>(&update_[0]) 数组转为vector3d
        _estimate.so3() = _estimate.so3() * SO3::exp(Eigen::Map<const Vec3d>(&update_[0]));
        // 平移部分的更新
        _estimate.translation() = _estimate.translation() + Eigen::Map<const Vec3d>(&update_[3]);
        updateCache();
    }
};

class VertexVelocity : public g2o::BaseVertex<3, Vec3d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {
    }

    virtual bool read(std::istream& is) {
        return false;
    }
    virtual bool write(std::ostream& os) const {
        return false;
    }

    virtual void setToOriginImpl() {
        _estimate.setZero();
    }

    virtual void oplusImpl(const double* update_) {
        _estimate += Eigen::Map<const Vec3d>(update_);
    }
};

/**
 * 陀螺零偏顶点，亦为Vec3d，从速度顶点继承
 */
class VertexGyroBias : public VertexVelocity {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias() {
    }
};

/**
 * 陀螺零偏顶点，亦为Vec3d，从速度顶点继承
 */
class VertexAcceBias : public VertexVelocity {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAcceBias() {
    }
};

/**
 * @brief 定义两个陀螺仪的零偏约束
 * @param int 测量数据的维度
 * @param Vec3d 数据的类型
 * @param VertexGyroBias 节点的类型
 */
class EdgeGyroRW : public g2o::BaseBinaryEdge<3, Vec3d, VertexGyroBias, VertexGyroBias> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyroRW() {
    }
    virtual bool read(std::istream& is) {
        return false;
    }
    virtual bool write(std::ostream& os) const {
        return false;
    }
    // 计算残差
    void computeError() {
        const auto* vertex1 = dynamic_cast<const VertexGyroBias*>(_vertices[0]);
        const auto* vertex2 = dynamic_cast<const VertexGyroBias*>(_vertices[1]);
        // 残差 bgj - bgi
        _error = vertex2->estimate() - vertex1->estimate();
    }

    // 计算雅可比矩阵
    virtual void linearizeOplus() {
        // _error = bgi - bgj
        // 对bgi的jacobian
        _jacobianOplusXi = -Mat3d::Identity();
        // 对bgj的jacobian
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

/**
 * @brief 定义两个陀螺仪的零偏约束
 * @param int 测量数据的维度
 * @param Vec3d 数据的类型
 * @param VertexGyroBias 节点的类型
 */
class EdgeAcceRW : public g2o::BaseBinaryEdge<3, Vec3d, VertexAcceBias, VertexAcceBias> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAcceRW() {
    }
    virtual bool read(std::istream& is) {
        return false;
    }
    virtual bool write(std::ostream& os) const {
        return false;
    }
    // 计算残差
    void computeError() {
        const auto* vertex1 = dynamic_cast<const VertexAcceBias*>(_vertices[0]);
        const auto* vertex2 = dynamic_cast<const VertexAcceBias*>(_vertices[1]);
        // 残差 bgj - bgi
        _error = vertex2->estimate() - vertex1->estimate();
    }

    // 计算雅可比矩阵
    virtual void linearizeOplus() {
        // _error = bgi - bgj
        // 对bgi的jacobian
        _jacobianOplusXi = -Mat3d::Identity();
        // 对bgj的jacobian
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

/**
 * 对上一帧IMU pvq bias的先验
 * info 由外部指定，通过时间窗口边缘化给出
 *
 * 顶点顺序：pose, v, bg, ba
 * 残差顺序：R, p, v, bg, ba，15维
 */
class EdgePriorPoseNavState : public g2o::BaseMultiEdge<15, Vec15d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorPoseNavState(const NavStated& state, const Mat15d& info);

    virtual bool read(std::istream& is) {
        return false;
    }
    virtual bool write(std::ostream& os) const {
        return false;
    }

    void computeError();
    virtual void linearizeOplus();

    Eigen::Matrix<double, 15, 15> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 15, 15> J;
        J.block<15, 6>(0, 0) = _jacobianOplus[0];
        J.block<15, 3>(0, 6) = _jacobianOplus[1];
        J.block<15, 3>(0, 9) = _jacobianOplus[2];
        J.block<15, 3>(0, 12) = _jacobianOplus[3];
        return J.transpose() * information() * J;
    }

    NavStated state_;
};

class EdgeGNSS : public g2o::BaseUnaryEdge<6, SE3, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(std::istream& in) {
        return true;
    }
    virtual bool write(std::ostream& out) const {
        return true;
    }
    EdgeGNSS() = default;
    EdgeGNSS(VertexPose* vertex, const SE3& obs) {
        // 边设置节点，vertex里面包含了状态量的值
        setVertex(0, vertex);
        // gnss对位置的绝对观测
        setMeasurement(obs);
    }
    void computeError() override {
        VertexPose* vertex = dynamic_cast<VertexPose*>(_vertices[0]);
        // R,p的顺序
        // 李群->李代数
        _error.head<3>() = (_measurement.so3().inverse() * vertex->estimate().so3()).log();
        // 预测值-观测值
        _error.tail<3>() = (vertex->estimate().translation() - _measurement.translation());
    }
    void linearizeOplus() override {
        VertexPose* vertex = dynamic_cast<VertexPose*>(_vertices[0]);
        _jacobianOplusXi.setZero();
        // derror_R / dR  这里的R是vector的R
        _jacobianOplusXi.block<3, 3>(0, 0) = (_measurement.so3().inverse() * vertex->estimate().so3()).jr_inv();
        // derror_R / dp  这里的R是vector的p = Zero();
        // derror_p / dR = Zero();
        // derror_p / dp
        _jacobianOplusXi.block<3, 3>(3, 3) = Mat3d::Identity();
    }
    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }
};

class EdgeGNSSTransOnly : public g2o::BaseUnaryEdge<3, Vec3d, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * 指定位姿顶点、RTK观测 t_WG、外参TGB
     * @param v
     * @param obs
     */
    EdgeGNSSTransOnly(VertexPose* v, const Vec3d& obs, const SE3& TBG) : TBG_(TBG) {
        setVertex(0, v);
        setMeasurement(obs);
    }

    void computeError() override {
        VertexPose* v = (VertexPose*)_vertices[0];
        _error = (v->estimate() * TBG_).translation() - _measurement;
    };

    // void linearizeOplus() override {
    //     VertexPose* v = (VertexPose*)_vertices[0];
    //     // jacobian 6x6
    //     _jacobianOplusXi.setZero();
    //     _jacobianOplusXi.block<3, 3>(0, 0) = (_measurement.so3().inverse() * v->estimate().so3()).jr_inv();  // dR/dR
    //     _jacobianOplusXi.block<3, 3>(3, 3) = Mat3d::Identity();                                              // dp/dp
    // }

    virtual bool read(std::istream& in) {
        return true;
    }
    virtual bool write(std::ostream& out) const {
        return true;
    }

   private:
    SE3 TBG_;
};

/**
 * @brief 提供两帧之间的里程计约束
 */
class EdgeRelativeMotion : public g2o::BaseBinaryEdge<6, SE3, VertexPose, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeRelativeMotion() = default;
    EdgeRelativeMotion(VertexPose* vertex1, VertexPose* vertex2, const SE3& relative_pose) {
        setVertex(0, vertex1);
        setVertex(1, vertex2);
    }
    virtual bool read(std::istream& is) override {
        double data[7];
        for (int i = 0; i < 7; i++) {
            is >> data[i];
        }
        Quatd q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(SE3(q, Vec3d(data[0], data[1], data[2])));
        for (int i = 0; i < information().rows() && is.good(); i++) {
            for (int j = i; j < information().cols() && is.good(); j++) {
                is >> information()(i, j);
                if (i != j) information()(j, i) = information()(i, j);
            }
        }
        return true;
    }

    virtual bool write(std::ostream& os) const override {
        os << "EDGE_SE3:QUAT ";
        auto* v1 = static_cast<VertexPose*>(_vertices[0]);
        auto* v2 = static_cast<VertexPose*>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        SE3 m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // information matrix
        for (int i = 0; i < information().rows(); i++) {
            for (int j = i; j < information().cols(); j++) {
                os << information()(i, j) << " ";
            }
        }
        os << std::endl;
        return true;
    }

    void computeError() override {
        VertexPose* v1 = (VertexPose*)_vertices[0];
        VertexPose* v2 = (VertexPose*)_vertices[1];
        SE3 T12 = v1->estimate().inverse() * v2->estimate();
        _error = (_measurement.inverse() * v1->estimate().inverse() * v2->estimate()).log();
    }
    // 不重写这个函数，g2o将调用自己的数值求导
    // void linearizeOplus() override {
    //     VertexPose* vertex1 = dynamic_cast<VertexPose*>(_vertices[0]);
    //     VertexPose* verte2 = dynamic_cast<VertexPose*>(_vertices[1]);
    //     _jacobianOplusXi.setZero();
    //     // derror_R / dRi  这里的R是vector的R
    //     _jacobianOplusXi.block<3, 3>(0, 0) = Mat3d::Zero();
    //     // derror_R / dpi
    //     _jacobianOplusXi.block<3, 3>(0, 3) = Mat3d::Zero();
    //     // derror_p / dRj
    //     _jacobianOplusXi.block<3, 3>(3, 0) = Mat3d::Zero();
    //     // derror_p / dpj
    //     _jacobianOplusXi.block<3, 3>(3, 3) = Mat3d::Zero();
    // }
    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }
};

/**
 * 3维 轮速计观测边
 * 轮速观测世界速度在自车坐标系下矢量, 3维情况下假设自车不会有y和z方向速度
 */
class EdgeEncoder3D : public g2o::BaseUnaryEdge<3, Vec3d, VertexVelocity> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeEncoder3D() = default;

    /**
     * 构造函数需要知道世界系下速度
     * @param v0
     * @param speed
     */
    EdgeEncoder3D(VertexVelocity* v0, const Vec3d& speed) {
        setVertex(0, v0);
        setMeasurement(speed);
    }

    void computeError() override {
        VertexVelocity* v0 = (VertexVelocity*)_vertices[0];
        _error = v0->estimate() - _measurement;
    }

    void linearizeOplus() override {
        _jacobianOplusXi.setIdentity();
    }

    virtual bool read(std::istream& in) {
        return true;
    }
    virtual bool write(std::ostream& out) const {
        return true;
    }
};

}  // namespace slam_learn