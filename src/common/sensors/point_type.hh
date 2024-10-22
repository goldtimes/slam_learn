#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>
#include <vector>
#include "common/eigen_types.hh"

namespace slam_learn {
using PointXYZI = pcl::PointXYZI;
using PointCloudXYZI = pcl::PointCloud<PointXYZI>;
using CloudPtr = PointCloudXYZI::Ptr;
using PointVec = std::vector<PointXYZI, Eigen::aligned_allocator<PointXYZI>>;
using IndexVec = std::vector<int>;
/// ui中的点云颜色
using UiPointType = pcl::PointXYZRGBA;
using UiPointCloudType = pcl::PointCloud<UiPointType>;
using UiCloudPtr = UiPointCloudType::Ptr;

/// 带ring, range等其他信息的全量信息点云
struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    double time = 0;
    float height = 0;

    inline FullPointType() {
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 全量点云的定义
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

}  // namespace slam_learn

// 点云注册

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(slam_learn::FullPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, radius, radius)(
                                      std::uint8_t, intensity, intensity)(std::uint16_t, angle, angle)(
                                      std::uint8_t, ring, ring)(double, time, time)(float, height, height))
// clang-format on
namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))
// clang-format on

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                      (std::uint32_t, t, t)
                                      (std::uint16_t, reflectivity, reflectivity)
                                      (std::uint8_t, ring, ring)
                                      (std::uint16_t, ambient, ambient)
                                      (std::uint32_t, range, range)
)
// clang-format on