#include "lidar_slam/ch7/loam-like/loam_like_odom.hh"
#include <pcl/common/transforms.h>
#include <algorithm>
#include <cmath>
#include <execution>
#include <vector>
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/sensors/point_type.hh"
#include "lidar_slam/ch7/loam-like/feature_extraction.hh"

namespace slam_learn::loam {
LoamLikeOdom::LoamLikeOdom(Options options)
    : options_(options), feature_extraction_(new FeatureExtraction), global_map_(new PointCloudXYZI()) {
    if (options_.display_realtime_cloud_) {
        viewer_ = std::make_shared<PCLMapViewer>(0.1);
    }

    kdtree_edge_.SetEnableANN();
    kdtree_surf_.SetEnableANN();
}
void LoamLikeOdom::ProcessPointCloud(FullCloudPtr full_cloud) {
    LOG(INFO) << "process frame:" << cnt_frame_++;
    // 特征提取
    CloudPtr current_edge(new PointCloudXYZI());
    CloudPtr current_surf(new PointCloudXYZI());
    feature_extraction_->Extract(full_cloud, current_edge, current_surf);
    if (current_edge->size() < options_.min_edge_pts_ || current_surf->size() < options_.min_surf_pts_) {
        LOG(ERROR) << "not enough edge/surf pts: " << current_edge->size() << "," << current_surf->size();
        return;
    }
    LOG(INFO) << "edge: " << current_edge->size() << ", surf: " << current_surf->size();
    // 第一帧的处理
    if (submap_edge_ == nullptr || submap_surf_ == nullptr) {
        submap_edge_ = current_edge;
        submap_surf_ = current_surf;

        if (options_.use_pcl_kdtree_) {
            pcl_kdtree_edge_.setInputCloud(submap_edge_);
            pcl_kdtree_surf_.setInputCloud(submap_surf_);
        } else {
            kdtree_edge_.BuildTree(submap_edge_);
            kdtree_surf_.BuildTree(submap_surf_);
        }
        edges_keyframes_buffer_.emplace_back(current_edge);
        surf_keyframes_buffer_.emplace_back(current_surf);
        return;
    }
    // 与局部地图配准
    SE3 pose = AlignWithLocalMap(current_edge, current_surf);
    CloudPtr scan_world(new PointCloudXYZI);
    pcl::transformPointCloud(*lidar_utils::ConvertToCloud<FullPointType>(full_cloud), *scan_world, pose.matrix());
    CloudPtr edge_world(new PointCloudXYZI);
    CloudPtr surf_world(new PointCloudXYZI);
    pcl::transformPointCloud(*current_edge, *edge_world, pose.matrix().cast<float>());
    pcl::transformPointCloud(*current_surf, *surf_world, pose.matrix().cast<float>());

    if (IsKeyframe(pose)) {
        LOG(INFO) << "inserting keyframe";
        last_kf_pose_ = pose;
        last_kf_id_ = cnt_frame_;

        edges_keyframes_buffer_.emplace_back(current_edge);
        surf_keyframes_buffer_.emplace_back(current_surf);
        if (edges_keyframes_buffer_.size() > options_.num_kfs_in_local_map_) {
            edges_keyframes_buffer_.pop_front();
        }
        if (surf_keyframes_buffer_.size() > options_.num_kfs_in_local_map_) {
            surf_keyframes_buffer_.pop_front();
        }
        submap_surf_.reset(new PointCloudXYZI);
        submap_edge_.reset(new PointCloudXYZI);

        for (const auto& cloud : edges_keyframes_buffer_) {
            *submap_edge_ += *cloud;
        }
        for (const auto& cloud : surf_keyframes_buffer_) {
            *submap_surf_ += *cloud;
        }

        submap_surf_ = lidar_utils::VoxelCloud(submap_surf_, 1.0);
        submap_edge_ = lidar_utils::VoxelCloud(submap_edge_, 1.0);

        LOG(INFO) << "insert keyframe, surf pts: " << submap_surf_->size() << ", edge pts: " << submap_edge_->size();

        if (options_.use_pcl_kdtree_) {
            pcl_kdtree_edge_.setInputCloud(submap_edge_);
            pcl_kdtree_surf_.setInputCloud(submap_surf_);
        } else {
            kdtree_edge_.BuildTree(submap_edge_);
            kdtree_surf_.BuildTree(submap_surf_);
        }
        *global_map_ += *scan_world;
    }

    LOG(INFO) << "current pose: " << pose.translation().transpose() << ", "
              << pose.so3().unit_quaternion().coeffs().transpose();

    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(pose, scan_world);
    }
}

/// 与局部地图进行配准
SE3 LoamLikeOdom::AlignWithLocalMap(CloudPtr edge, CloudPtr surf) {
    SE3 pose;
    if (estimated_poses_.size() >= 2) {
        // 从最近两个pose来推断
        SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
        SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
        pose = T1 * (T2.inverse() * T1);
    }
    int edge_size = edge->size();
    int surf_size = surf->size();
    // 点到面icp
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        std::vector<bool> effect_surf(surf_size, false);
        std::vector<Eigen::Matrix<double, 1, 6>> jacobians_surf(surf_size);
        std::vector<double> errors_surf(surf_size);

        std::vector<bool> effect_edge(edge_size, false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacob_edge(edge_size);  // 点线的残差是3维的
        std::vector<Vec3d> errors_edge(edge_size);

        std::vector<int> index_surf(surf_size);
        std::iota(index_surf.begin(), index_surf.end(), 0);  // 填入
        std::vector<int> index_edge(edge_size);
        std::iota(index_edge.begin(), index_edge.end(), 0);  // 填入

        // icp
        if (options_.use_edge_points_) {
            std::for_each(std::execution::par_unseq, index_edge.begin(), index_edge.end(), [&](const int idx) {
                Vec3d point_origin = lidar_utils::ToVec3d(edge->points[idx]);
                auto point_transed = pose * point_origin;
                std::vector<int> nn_indices;
                std::vector<float> nn_dist;
                if (options_.use_pcl_kdtree_) {
                    pcl_kdtree_edge_.nearestKSearch(lidar_utils::ToPointType(point_transed), 5, nn_indices, nn_dist);
                } else {
                    kdtree_edge_.GetClosestPoint(lidar_utils::ToPointType(point_transed), nn_indices);
                }
                effect_edge[idx] = false;
                if (nn_indices.size() >= 3) {
                    std::vector<Vec3d> nn_eigen;
                    for (auto& n : nn_indices) {
                        nn_eigen.push_back(lidar_utils::ToVec3d(submap_edge_->points[n]));
                    }
                    // 拟合直线
                    Vec3d d;
                    Vec3d p0;
                    if (!math::FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
                        return;
                    }
                    Vec3d err = SO3::hat(d) * (point_transed - p0);
                    if (err.norm() > options_.max_line_distance_) {
                        return;
                    }
                    effect_edge[idx] = true;
                    // jacobians
                    // build residual
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -SO3::hat(d) * pose.so3().matrix() * SO3::hat(point_origin);
                    J.block<3, 3>(0, 3) = SO3::hat(d);

                    jacob_edge[idx] = J;
                    errors_edge[idx] = err;
                }
            });
        }

        if (options_.use_surf_points_) {
            std::for_each(std::execution::par_unseq, index_surf.begin(), index_surf.end(), [&](const int idx) {
                Vec3d point_origin = lidar_utils::ToVec3d(surf->points[idx]);
                auto point_transed = pose * point_origin;
                std::vector<int> nn_indices;
                std::vector<float> nn_dist;
                if (options_.use_pcl_kdtree_) {
                    pcl_kdtree_surf_.nearestKSearch(lidar_utils::ToPointType(point_transed), 5, nn_indices, nn_dist);
                } else {
                    kdtree_surf_.GetClosestPoint(lidar_utils::ToPointType(point_transed), nn_indices);
                }
                effect_surf[idx] = false;
                if (nn_indices.size() == 5) {
                    std::vector<Vec3d> nn_eigen;
                    for (auto& n : nn_indices) {
                        nn_eigen.push_back(lidar_utils::ToVec3d(submap_surf_->points[n]));
                    }
                    // 拟合平面
                    Vec4d n;
                    if (!math::FitPlane(nn_eigen, n)) {
                        return;
                    }
                    double dist = n.head<3>().dot(point_transed) + n[3];
                    if (fabs(dist) > options_.max_plane_distance_) {
                        return;
                    }
                    effect_surf[idx] = true;
                    // jacobians
                    // build residual
                    Eigen::Matrix<double, 1, 6> J;
                    J.block<1, 3>(0, 0) = -n.head<3>().transpose() * pose.so3().matrix() * SO3::hat(point_origin);
                    J.block<1, 3>(0, 3) = n.head<3>().transpose();

                    jacobians_surf[idx] = J;
                    errors_surf[idx] = dist;
                }
            });
        }
        // 累加Hessian和error,计算dx
        // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
        double total_res = 0;
        int effective_num = 0;

        Mat6d H = Mat6d::Zero();
        Vec6d err = Vec6d::Zero();

        for (const auto& idx : index_surf) {
            if (effect_surf[idx]) {
                H += jacobians_surf[idx].transpose() * jacobians_surf[idx];
                err += -jacobians_surf[idx].transpose() * errors_surf[idx];
                effective_num++;
                total_res += errors_surf[idx] * errors_surf[idx];
            }
        }

        for (const auto& idx : index_edge) {
            if (effect_edge[idx]) {
                H += jacob_edge[idx].transpose() * jacob_edge[idx];
                err += -jacob_edge[idx].transpose() * errors_edge[idx];
                effective_num++;
                total_res += errors_edge[idx].norm();
            }
        }

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return pose;
        }

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }
    estimated_poses_.emplace_back(pose);
    return pose;
}

/// 判定是否为关键帧
bool LoamLikeOdom::IsKeyframe(const SE3& current_pose) {
    if ((cnt_frame_ - last_kf_id_) > 30) {
        return true;
    }
    // 只要与上一帧相对运动超过一定距离或角度，就记关键帧
    SE3 delta = last_kf_pose_.inverse() * current_pose;
    return delta.translation().norm() > options_.kf_distance_ ||
           delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
}

void LoamLikeOdom::SaveMap(const std::string& path) {
    if (global_map_ && global_map_->empty() == false) {
        lidar_utils::SaveCloudToFile(path, *global_map_);
    }
}
}  // namespace slam_learn::loam