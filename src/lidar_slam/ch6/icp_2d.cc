#include "lidar_slam/ch6/icp_2d.hh"
#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>
#include "common/eigen_types.hh"
#include "common/math_utils.hh"

namespace slam_learn::icp_2d {
void Icp2d::BuildTargetKdTree() {
    if (target_scan_ == nullptr) {
        LOG(ERROR) << "target is not set";
        return;
    }
    // scan->to-cloud
    target_cloud_.reset(new Cloud2d);
    for (size_t i = 0; i < target_scan_->ranges.size(); ++i) {
        if (target_scan_->ranges[i] < target_scan_->range_min || target_scan_->ranges[i] > target_scan_->range_max) {
            continue;
        }

        double real_angle = target_scan_->angle_min + i * target_scan_->angle_increment;

        Point2d p;
        p.x = target_scan_->ranges[i] * std::cos(real_angle);
        p.y = target_scan_->ranges[i] * std::sin(real_angle);
        target_cloud_->points.push_back(p);
    }

    target_cloud_->width = target_cloud_->points.size();
    target_cloud_->is_dense = false;
    kdtree_.setInputCloud(target_cloud_);
}

// 高斯牛顿的配准方式
bool Icp2d::AlignGaussNewton(SE2& init_pose) {
    // 定义迭代的次数
    int iterations = 10;
    // 定义每次的cost和记录下一次的cost
    double cost = 0.0, last_cost = 0.0;
    SE2 current_pose = init_pose;
    // 最近的阈值
    const float max_dist2 = 0.01;
    const int min_effect_pts = 20;
    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Identity();
        Vec3d b = Vec3d::Zero();
        cost = 0.0;
        int effective_num = 0;
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            // 将雷达坐标系的点转到世界坐标系
            float r = source_scan_->ranges[i];
            if (r < source_scan_->range_min || r > source_scan_->range_max) {
                continue;
            }

            float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
            float theta = current_pose.so2().log();
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();
            // 最近邻
            std::vector<int> nn_idx;
            std::vector<float> dists;
            kdtree_.nearestKSearch(pt, 1, nn_idx, dists);
            if (nn_idx.size() > 0 && dists[0] < max_dist2) {
                effective_num++;
                // 点到点的jacobians 2x3的矩阵
                Mat32d J;
                J << 1, 0, 0, 1, -r * sin(angle + theta), r * cos(angle + theta);
                H += J * J.transpose();

                Vec2d error(pt.x - target_cloud_->points[nn_idx[0]].x, pt.y - target_cloud_->points[nn_idx[0]].y);
                b += -J * error;

                cost += error.dot(error);
            }
        }
        if (effective_num < min_effect_pts) {
            return false;
        }
        // solve for dx
        Vec3d dx = H.ldlt().solve(b);
        if (std::isnan(dx[0])) {
            break;
        }
        cost /= effective_num;
        if (iter > 0 && cost >= last_cost) {
            break;
        }
        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;
        // 更新pose
        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        last_cost = cost;
    }
    init_pose = current_pose;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
              << ", theta: " << current_pose.so2().log();

    return true;
}

bool Icp2d::AlignGaussNewtonPoint2Line(SE2& init_pose) {
    // 定义迭代的次数
    int iterations = 10;
    // 定义每次的cost和记录下一次的cost
    double cost = 0.0, last_cost = 0.0;
    SE2 current_pose = init_pose;
    // 最近的阈值
    const float max_dist2 = 0.3;
    const int min_effect_pts = 20;
    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Identity();
        Vec3d b = Vec3d::Zero();
        cost = 0.0;
        int effective_num = 0;
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            // 将雷达坐标系的点转到世界坐标系
            float r = source_scan_->ranges[i];
            if (r < source_scan_->range_min || r > source_scan_->range_max) {
                continue;
            }

            float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
            float theta = current_pose.so2().log();
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();
            // 最近邻
            std::vector<int> nn_idx;
            std::vector<float> dists;
            kdtree_.nearestKSearch(pt, 5, nn_idx, dists);
            std::vector<Vec2d> effective_pts;
            for (int j = 0; j < nn_idx.size(); ++j) {
                if (dists[j] < max_dist2) {
                    effective_pts.emplace_back(
                        Vec2d(target_cloud_->points[nn_idx[j]].x, target_cloud_->points[nn_idx[j]].y));
                }
            }
            if (effective_pts.size() < 3) {
                continue;
            }
            // 拟合直线
            Vec3d line_coeffs;
            if (math::FitLine2D(effective_pts, line_coeffs)) {
                effective_num++;
                Vec3d J;
                J << line_coeffs[0], line_coeffs[1],
                    -line_coeffs[0] * r * std::sin(angle + theta) + line_coeffs[1] * r * std::cos(angle + theta);
                H += J * J.transpose();

                double error = line_coeffs[0] * pt.x + line_coeffs[1] * pt.y + line_coeffs[2];
                b += -J * error;
                cost += error * error;
            }
        }
        if (effective_num < min_effect_pts) {
            return false;
        }
        // solve for dx
        Vec3d dx = H.ldlt().solve(b);
        if (std::isnan(dx[0])) {
            break;
        }
        cost /= effective_num;
        if (iter > 0 && cost >= last_cost) {
            break;
        }
        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;
        // 更新pose
        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        last_cost = cost;
    }
    init_pose = current_pose;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
              << ", theta: " << current_pose.so2().log();

    return true;
}
// LM算法的配准方式
bool Icp2d::AlignLM(SE2& init_pose) {
}
bool Icp2d::AlignLMPoint2Line(SE2& init_pose) {
}
}  // namespace slam_learn::icp_2d