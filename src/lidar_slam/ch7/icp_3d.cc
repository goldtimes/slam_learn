#include "lidar_slam/ch7/icp_3d.hh"
#include <glog/logging.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <execution>
#include <vector>
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"

namespace slam_learn::icp_3d {
/// 使用gauss-newton方法进行配准, 点到点
bool Icp3d::AlignP2P(SE3& init_pose) {
    LOG(INFO) << "AlignP2P";
    assert(target_ != nullptr && source_ != nullptr);
    SE3 pose = init_pose;
    if (!options_.use_initial_translation_) {
        pose.translation() = target_center_ - source_center_;
    }
    std::vector<int> index(source_->size());
    std::for_each(index.begin(), index.end(), [id = 0](int& i) mutable { i = id++; });

    std::vector<bool> effect_pts(source_->size(), false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(source_->size());
    std::vector<Vec3d> errors(source_->size());
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // 并发遍历所有的点 p2p
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto point_origin = lidar_utils::ToVec3d(source_->points[idx]);
            Vec3d point_transed = pose * point_origin;
            // 最邻近
            std::vector<int> nn_idx;
            std::vector<float> dists;
            kdtree_.nearestKSearch(lidar_utils::ToPointType(point_transed), 1, nn_idx, dists);
            if (!nn_idx.empty()) {
                // 判断距离
                Vec3d point_searched = lidar_utils::ToVec3d(target_->points[nn_idx[0]]);
                double dist2 = (point_searched - point_transed).squaredNorm();
                // 不符合阈值
                if (dist2 > options_.max_nn_distance_) {
                    effect_pts[idx] = false;
                    return;
                }
                effect_pts[idx] = true;
                // 构建点到点的残差
                Vec3d error = (point_searched - point_transed);
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = pose.so3().matrix() * SO3::hat(point_origin);
                J.block<3, 3>(0, 3) = -Mat3d::Identity();
                jacobians[idx] = J;
                errors[idx] = error;
            } else {
                effect_pts[idx] = false;
            }
        });
        // 计算H,b矩阵，判断收敛和更新位姿
        double total_res = 0.0;
        int effective_num = 0;
        Mat6d H = Mat6d::Zero();
        Vec6d b = Vec6d::Zero();
        for (int i = 0; i < effect_pts.size(); ++i) {
            if (!effect_pts[i]) {
                continue;
            } else {
                H += jacobians[i].transpose() * jacobians[i];
                total_res += errors[i].dot(errors[i]);
                effective_num++;
                b += -jacobians[i].transpose() * errors[i];
            }
        }
        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }
        Vec6d dx = H.inverse() * b;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();
        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }
    init_pose = pose;
    return true;
}

/// 基于gauss-newton的点线ICP
bool Icp3d::AlignP2Line(SE3& init_pose) {
    LOG(INFO) << "AlignP2Line";
    assert(target_ != nullptr && source_ != nullptr);
    SE3 pose = init_pose;
    if (!options_.use_initial_translation_) {
        pose.translation() = target_center_ - source_center_;
    }
    std::vector<int> index(source_->size());
    std::for_each(index.begin(), index.end(), [id = 0](int& i) mutable { i = id++; });

    std::vector<bool> effect_pts(source_->size(), false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(source_->size());
    std::vector<Vec3d> errors(source_->size());
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // 并发遍历所有的点 p2line
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto point_origin = lidar_utils::ToVec3d(source_->points[idx]);
            Vec3d point_transed = pose * point_origin;
            // 最邻近
            std::vector<int> nn_idx;
            std::vector<float> dists;
            kdtree_.nearestKSearch(lidar_utils::ToPointType(point_transed), 5, nn_idx, dists);
            if (nn_idx.size() == 5) {
                // 判断距离
                std::vector<Vec3d> nn_eigen_pt;
                for (int i = 0; i < nn_idx.size(); ++i) {
                    Vec3d point_searched = lidar_utils::ToVec3d(target_->points[nn_idx[i]]);
                    nn_eigen_pt.emplace_back(point_searched);
                }
                // 平面拟合,直线的起点和直线的方向
                Vec3d d, p0;
                // 不符合阈值
                if (!math::FitLine(nn_eigen_pt, p0, d, options_.max_line_distance_)) {
                    effect_pts[idx] = false;
                    return;
                }
                // 构建点到线的残差
                Vec3d error = SO3::hat(d) * (point_transed - p0);
                if (error.norm() > options_.max_line_distance_) {
                    effect_pts[idx] = false;
                    return;
                }
                effect_pts[idx] = true;
                // jacobian
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = -SO3::hat(d) * pose.so3().matrix() * SO3::hat(point_origin);
                J.block<3, 3>(0, 3) = SO3::hat(d);

                jacobians[idx] = J;
                errors[idx] = error;

            } else {
                effect_pts[idx] = false;
            }
        });
        // 计算H,b矩阵，判断收敛和更新位姿
        double total_res = 0.0;
        int effective_num = 0;
        Mat6d H = Mat6d::Zero();
        Vec6d b = Vec6d::Zero();
        for (int i = 0; i < effect_pts.size(); ++i) {
            if (!effect_pts[i]) {
                continue;
            } else {
                H += jacobians[i].transpose() * jacobians[i];
                total_res += errors[i].dot(errors[i]);
                effective_num++;
                b += -jacobians[i].transpose() * errors[i];
            }
        }
        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }
        Vec6d dx = H.inverse() * b;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();
        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }
    init_pose = pose;
    return true;
}

/// 基于gauss-newton的点面ICP
bool Icp3d::AlignP2Plane(SE3& init_pose) {
    LOG(INFO) << "AlignP2Plane";
    assert(target_ != nullptr && source_ != nullptr);
    SE3 pose = init_pose;
    if (!options_.use_initial_translation_) {
        pose.translation() = target_center_ - source_center_;
    }
    std::vector<int> index(source_->size());
    std::for_each(index.begin(), index.end(), [id = 0](int& i) mutable { i = id++; });

    std::vector<bool> effect_pts(source_->size(), false);
    std::vector<Eigen::Matrix<double, 1, 6>> jacobians(source_->size());
    std::vector<double> errors(source_->size());
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // 并发遍历所有的点 p2Plane
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto point_origin = lidar_utils::ToVec3d(source_->points[idx]);
            Vec3d point_transed = pose * point_origin;
            // 最邻近
            std::vector<int> nn_idx;
            std::vector<float> dists;
            kdtree_.nearestKSearch(lidar_utils::ToPointType(point_transed), 5, nn_idx, dists);
            if (nn_idx.size() > 3) {
                // 判断距离
                std::vector<Vec3d> nn_eigen_pt;
                for (int i = 0; i < nn_idx.size(); ++i) {
                    Vec3d point_searched = lidar_utils::ToVec3d(target_->points[nn_idx[i]]);
                    nn_eigen_pt.emplace_back(point_searched);
                }
                // 平面拟合,直线的起点和直线的方向
                Vec4d plane_coeffs;
                // 不符合阈值
                if (!math::FitPlane(nn_eigen_pt, plane_coeffs)) {
                    effect_pts[idx] = false;
                    return;
                }
                // 构建点到面的残差
                double error = plane_coeffs.head<3>().dot(point_transed) + plane_coeffs[3];
                if (std::fabs(error) > options_.max_plane_distance_) {
                    effect_pts[idx] = false;
                    return;
                }
                effect_pts[idx] = true;
                // jacobian
                Eigen::Matrix<double, 1, 6> J;
                J.block<1, 3>(0, 0) =
                    -plane_coeffs.head<3>().transpose() * pose.so3().matrix() * SO3::hat(point_origin);
                J.block<1, 3>(0, 3) = plane_coeffs.head<3>();

                jacobians[idx] = J;
                errors[idx] = error;
            } else {
                effect_pts[idx] = false;
            }
        });
        // 计算H,b矩阵，判断收敛和更新位姿
        double total_res = 0.0;
        int effective_num = 0;
        Mat6d H = Mat6d::Zero();
        Vec6d b = Vec6d::Zero();
        for (int i = 0; i < effect_pts.size(); ++i) {
            if (!effect_pts[i]) {
                continue;
            } else {
                H += jacobians[i].transpose() * jacobians[i];
                total_res += errors[i] * (errors[i]);
                effective_num++;
                b += -jacobians[i].transpose() * errors[i];
            }
        }
        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }
        Vec6d dx = H.inverse() * b;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();
        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }
    init_pose = pose;
    return true;
}

void Icp3d::BuildTargetTree() {
    kdtree_.setInputCloud(target_);
}
}  // namespace slam_learn::icp_3d