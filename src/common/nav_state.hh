#pragma once
#include <iostream>
#include "common/eigen_types.hh"

namespace slam_learn {
template <typename T>
class NavState {
   public:
    NavState() = default;
    explicit NavState(double time, const SO3& R = SO3(), const Vec3d& p = Vec3d::Zero(), const Vec3d& v = Vec3d::Zero(),
                      const Vec3d& bg = Vec3d::Zero(), const Vec3d& ba = Vec3d::Zero())
        : timestamped_(time), R_(R), p_(p), v_(v), bg_(bg), ba_(ba) {
    }
    NavState(double time, const SO3& R, const Vec3d& p, const Vec3d& v, const Vec3d& bg = Vec3d::Zero(),
             const Vec3d& ba = Vec3d::Zero())
        : timestamped_(time), R_(R), p_(p), v_(v), bg_(bg), ba_(ba) {
    }

    // from pose and vel
    NavState(double time, const SE3& pose, const Vec3& vel = Vec3::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {
    }

    Sophus::SE3<T> GetSE3() const {
        return SE3(R_, p_);
    }

    // 重载 << 输出
    friend std::ostream& operator<<(std::ostream& os, const NavState<T>& nav_state) {
        os << "p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
           << ", q: " << s.R_.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg_.transpose()
           << ", ba: " << s.ba_.transpose();
        return os;
    }

    double timestamped_ = 0;

    SO3 R_;
    Vec3d p_ = Vec3d::Zero();
    Vec3d v_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
};
}  // namespace slam_learn