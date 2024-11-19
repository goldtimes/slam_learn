#include "v_slam/ch5/code_ba_schur/backend/loss_function.hh"

namespace slam_learn::backend {
double HuberLoss::Compute(double error) const {
    if (error <= delta_) {
        return error * error;
    } else {
        return delta_ * (2 * error - delta_);
    }
}
}  // namespace slam_learn::backend