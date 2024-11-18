#include "v_slam/ch5/code_ba_schur/backend/imu_preintegration.hh"

namespace slam_learn::backend {
/**
 * propage pre-integrated measurements using raw IMU data
 * @param dt
 * @param acc
 * @param gyr_1
 */
void IMUIntegration::Propagate(double dt, const Vec3 &acc, const Vec3 &gyr) {
}

/**
 * according to pre-integration, when bias is updated, pre-integration should also be updated using
 * first-order expansion of ba and bg
 *
 * @param delta_ba
 * @param delta_bg
 */
void IMUIntegration::Correct(const Vec3 &delta_ba, const Vec3 &delta_bg) {
}

/// if bias is update by a large value, redo the propagation
void IMUIntegration::Repropagate() {
}
}  // namespace slam_learn::backend