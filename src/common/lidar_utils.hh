#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "common/eigen_types.hh"
#include "common/sensors/point_type.hh"
#include "velodyne_msgs/VelodyneScan.h"

/// 雷达扫描的一些消息定义和工具函数
using Scan2d = sensor_msgs::LaserScan;
using MultiScan2d = sensor_msgs::MultiEchoLaserScan;
using PacketsMsg = velodyne_msgs::VelodyneScan;
using PacketsMsgPtr = boost::shared_ptr<PacketsMsg>;

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

template <typename S>
inline PointXYZI ToPointType(const Eigen::Matrix<S, 3, 1>& pt) {
    PointXYZI point;
    point.x = pt[0];
    point.y = pt[1];
    point.z = pt[2];
    return point;
}

inline void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05) {
    pcl::VoxelGrid<PointXYZI> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);
    CloudPtr output(new PointCloudXYZI);
    voxel.filter(*output);
    cloud->swap(*output);
}

/// 对点云进行voxel filter,指定分辨率
inline CloudPtr VoxelCloud(CloudPtr cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PointXYZI> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudXYZI);
    voxel.filter(*output);
    return output;
}

inline CloudPtr PointCloud2ToCloudPtr(sensor_msgs::PointCloud2::Ptr msg) {
    CloudPtr cloud(new PointCloudXYZI);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
}

template <typename CloudType>
void SaveCloudToFile(const std::string& filePath, CloudType& cloud) {
    cloud.height = 1;
    cloud.width = cloud.size();
    pcl::io::savePCDFileASCII(filePath, cloud);
}

template <typename S, int n>
inline Eigen::Matrix<int, n, 1> CastToInt(const Eigen::Matrix<S, n, 1>& value) {
    return value.array().template round().template cast<int>();
}
}  // namespace slam_learn::lidar_utils