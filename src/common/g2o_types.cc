#include "common/g2o_types.hh"

namespace slam_learn {

EdgePriorPoseNavState::EdgePriorPoseNavState(const NavStated& state, const Mat15d& info) {
    resize(4);  // 4个顶点 T,v, bg, ba
    state_ = state;
    setInformation(info);
}
void EdgePriorPoseNavState::computeError() {
    auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* vv = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* vg = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* va = dynamic_cast<const VertexAcceBias*>(_vertices[3]);

    const Vec3d er = SO3(state_.R_.matrix().transpose() * vp->estimate().so3().matrix()).log();
    const Vec3d ep = vp->estimate().translation() - state_.p_;
    const Vec3d ev = vv->estimate() - state_.v_;
    const Vec3d ebg = vg->estimate() - state_.bg_;
    const Vec3d eba = va->estimate() - state_.ba_;
    // 15维
    _error << er, ep, ev, ebg, eba;
}
void EdgePriorPoseNavState::linearizeOplus() {
    const auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    const Vec3d er = SO3(state_.R_.matrix().transpose() * vp->estimate().so3().matrix()).log();
    // 整个jacobian的大小为15 * 15
    // 第一个jacobian矩阵的大小为6x6
    _jacobianOplus[0].setZero();
    // der / R
    _jacobianOplus[0].block<3, 3>(0, 0) = SO3::jr_inv(er);
    // der / p = Zero
    _jacobianOplus[0].block<3, 3>(0, 3) = Mat3d::Zero();
    // dep / R
    _jacobianOplus[0].block<3, 3>(3, 0) = Mat3d::Zero();
    // dep / p
    _jacobianOplus[0].block<3, 3>(3, 3) = Mat3d::Identity();
    _jacobianOplus[1].setZero();
    // dev / v
    _jacobianOplus[1].block<3, 3>(6, 0) = Mat3d::Identity();
    _jacobianOplus[2].setZero();
    // debg / dbg
    _jacobianOplus[2].block<3, 3>(9, 0) = Mat3d::Identity();
    _jacobianOplus[3].setZero();
    // deba / dba
    _jacobianOplus[3].block<3, 3>(12, 0) = Mat3d::Identity();
}
}  // namespace slam_learn