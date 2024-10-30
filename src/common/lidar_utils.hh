#pragma once

#include "common/sensors/point_type.hh"

namespace slam_learn::lidar_utils {
// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointXYZI& pt) {
    return pt.getVector3fMap();
}
inline Vec3d ToVec3d(const PointXYZI& pt) {
    return pt.getVector3fMap().cast<double>();
}
}  // namespace slam_learn::lidar_utils