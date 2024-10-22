#pragma once

#include "common/eigen_types.hh"
#include <memory>

namespace slam_learn {
struct Odom {
  Odom() {}
  Odom(double timestamp, double left_pulse, double right_pulse)
      : timestamp_(timestamp), left_pulse_(left_pulse),
        right_pulse_(right_pulse) {}

  double timestamp_ = 0.0;
  double left_pulse_ = 0.0; // 左右轮的单位时间转过的脉冲数
  double right_pulse_ = 0.0;
};
using OdomPtr = std::shared_ptr<Odom>;
} // namespace slam_learn