#include "lidar_slam/ch7//ndt_3d.hh"
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <glog/logging.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <execution>
#include <vector>
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"

namespace slam_learn::ndt_3d {
void Ndt3d::BuildVoxels() {
    assert(target_ != nullptr);
    assert(target_->empty() == false);
    grids_.clear();

    // 分配体素
    std::vector<size_t> index(target_->size());
    std::for_each(index.begin(), index.end(), [id = 0](size_t& i) mutable { i = id++; });
    // 构建体素
    std::for_each(index.begin(), index.end(), [this](const size_t& idx) {
        Vec3d pt = lidar_utils::ToVec3d(target_->points[idx]) * options_.inv_voxel_size_;
        // 这里没有*分辨率
        auto key = lidar_utils::CastToInt(pt);
        if (grids_.find(key) == grids_.end()) {
            grids_.insert({key, VoxelData(idx)});
        } else {
            grids_[key].idx_.emplace_back(idx);
        }
    });
    // 计算每个体素中的点云均值和方差
    std::for_each(std::execution::par_unseq, grids_.begin(), grids_.end(), [&](auto& gird) {
        // 点数太少的voxel直接跳过
        if (gird.second.idx_.size() > options_.min_pts_in_voxel_) {
            math::ComputeMeanAndCov(gird.second.idx_, gird.second.mu_, gird.second.sigma_,
                                    [&](const size_t& idx) { return lidar_utils::ToVec3d(target_->points[idx]); });
            // svd坚持最大最小的奇异值
            Eigen::JacobiSVD svd(gird.second.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3d lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3) {
                lambda[1] = lambda[0] * 1e-3;
            }
            if (lambda[2] < lambda[0] * 1e-3) {
                lambda[2] = lambda[0] * 1e-3;
            }
            Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
            gird.second.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        }
    });
    // 删除点数不够的体素，可以加快速度
    for (auto iter = grids_.begin(); iter != grids_.end();) {
        if (iter->second.idx_.size() > options_.min_pts_in_voxel_) {
            iter++;
        } else {
            // 删除map中的迭代器
            iter = grids_.erase(iter);
        }
    }
}
void Ndt3d::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

/// 使用gauss-newton方法进行ndt配准
bool Ndt3d::AlignNdt(SE3& init_pose) {
    LOG(INFO) << "aligning with ndt";
    assert(grids_.empty() == false);
    SE3 pose = init_pose;

    if (options_.remove_centroid_) {
        pose.translation() = target_center_ - source_center_;  // 设置平移初始值
        LOG(INFO) << "init trans set to " << pose.translation().transpose();
    }

    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6) {
        num_residual_per_point = 7;
    }

    std::vector<int> index(source_->size());
    std::for_each(index.begin(), index.end(), [id = 0](int& i) mutable { i = id++; });
    // 如果每个点只找一个体素，或者每个点遍历周围的6个体素和它本身
    int total_size = index.size() * num_residual_per_point;
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        std::vector<bool> effect_pts(total_size);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vec3d> errors(total_size);
        std::vector<Mat3d> infos(total_size);
        // 遍历所有source 点云
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const int idx) {
            auto point_origin = lidar_utils::ToVec3d(source_->points[idx]);
            Vec3d point_transed = pose * point_origin;
            // 计算变换后点的Key
            Vec3i key = lidar_utils::CastToInt(Vec3d(point_transed * options_.inv_voxel_size_));
            // 遍历周围的体素
            for (int i = 0; i < nearby_grids_.size(); ++i) {
                auto key_offset = key + nearby_grids_[i];
                auto grid_iter = grids_.find(key_offset);
                int real_idx = idx * num_residual_per_point + i;
                if (grid_iter != grids_.end()) {
                    auto& grid_value = grid_iter->second;
                    // 误差 = 值 - 均值
                    Vec3d e = point_transed - grid_value.mu_;
                    // 残差的定义
                    double res = e.transpose() * grid_value.info_ * e;
                    if (std::isnan(res) || res > options_.res_outlier_th_) {
                        effect_pts[real_idx] = false;
                        // 跳过这个体素
                        continue;
                    }

                    // 求解雅克比矩阵
                    Eigen::Matrix<double, 3, 6> J;
                    // de / dR
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(point_origin);
                    // de / dt
                    J.block<3, 3>(0, 3) = Mat3d::Identity();
                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = grid_value.info_;
                    effect_pts[real_idx] = true;
                } else {
                    effect_pts[real_idx] = false;
                }
            }
        });
        // H矩阵和b矩阵
        double total_res = 0;
        int effective_num = 0;
        Mat6d H = Mat6d::Zero();
        Vec6d b = Vec6d::Zero();
        for (int i = 0; i < effect_pts.size(); ++i) {
            // 无效的点
            if (!effect_pts[i]) {
                continue;
            }
            total_res += errors[i].transpose() * infos[i] * errors[i];
            effective_num++;
            H += jacobians[i].transpose() * infos[i] * jacobians[i];
            b += -jacobians[i].transpose() * infos[i] * errors[i];
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
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm()
                  << ", dx: " << dx.transpose();
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

/// 使用LM方法进行ndt配准
bool Ndt3d::AlignNdtLM(SE3& init_pose) {
}
}  // namespace slam_learn::ndt_3d