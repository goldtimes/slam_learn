#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "common/eigen_types.hh"
#include "common/sensors/point_type.hh"

namespace slam_learn::lidar_utils {
// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointXYZI& pt) {
    return pt.getVector3fMap();
}
inline Vec3d ToVec3d(const PointXYZI& pt) {
    return pt.getVector3fMap().cast<double>();
}

// 声明模板函数
template <typename T, int dim>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointXYZI& pt);

// 实现
template <>
inline Eigen::Matrix<float, 2, 1> ToEigen(const PointXYZI& pt) {
    return Vec2f(pt.x, pt.y);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen(const PointXYZI& pt) {
    return Vec3f(pt.x, pt.y, pt.z);
}

inline void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05) {
    pcl::VoxelGrid<PointXYZI> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);
    CloudPtr output(new PointCloudXYZI);
    voxel.filter(*output);
    cloud->swap(*output);
}
}  // namespace slam_learn::lidar_utils