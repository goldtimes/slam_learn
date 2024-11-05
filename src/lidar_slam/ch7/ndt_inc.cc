#include "lidar_slam/ch7/ndt_inc.hh"
#include <Eigen/src/Core/Matrix.h>
#include <glog/logging.h>
#include <algorithm>
#include <cassert>
#include <execution>
#include <numeric>
#include <set>
#include <utility>
#include "common/compare_func.hh"
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/timer/timer.hh"

namespace slam_learn::ndt_inc {
/// 在voxel里添加点云，
void IncNdt3d::AddCloud(CloudPtr cloud_world) {
    // 体素化点云，然后更新点云协方差
    // 记录哪些体素被更新了
    std::set<KeyType, less_vec<3>> active_voxels;
    for (const auto& point : cloud_world->points) {
        auto pt = lidar_utils::ToVec3d(point);
        auto key = lidar_utils::CastToInt(Vec3d(pt * options_.inv_voxel_size_));
        auto iter = grids_.find(key);
        if (iter == grids_.end()) {
            // 放到list中
            data_.push_front({key, VoxelData(pt)});
            // 创建新的体素
            grids_.insert({key, data_.begin()});
            if (data_.size() >= options_.capacity_) {
                // 删除尾部的数据
                grids_.erase(data_.back().first);
                data_.pop_back();
            }
        } else {
            // 体素中添加点
            iter->second->second.AddPoint(pt);
            // 将更新的这个Voxel放到list最前面
            data_.splice(data_.begin(), data_, iter->second);
            // grids_中的value也指向最前面
            iter->second = data_.begin();
        }
        active_voxels.emplace(key);
    }
    // 遍历更新所有active_voxel体素
    std::for_each(std::execution::par_unseq, active_voxels.begin(), active_voxels.end(),
                  [this](const auto& key) { UpdateVoxel(grids_[key]->second); });
    flag_first_scan_ = false;
}

void IncNdt3d::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

/// 更新体素内部数据, 根据新加入的pts和历史的估计情况来确定自己的估计
void IncNdt3d::UpdateVoxel(VoxelData& v) {
    // 首帧的数据来了之后，会将体素中的点估计一次得到均值和协方差
    // 然后清理掉pts_,等待后面的点进来
    if (flag_first_scan_) {
        if (v.pts_.size() > 1) {
            math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3d& pt) { return pt; });
            v.info_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse();
        } else {
            v.mu_ = v.pts_[0];
            v.info_ = Mat3d::Identity() * 1e2;
        }
        v.ndt_estimated_ = true;
        v.pts_.clear();
        return;
    }
    // 体素内点达到一定的数量之后，我们选择不更新了均值和协方差了
    if (v.ndt_estimated_ && v.num_pts_ > options_.max_pts_in_voxel_) {
        return;
    }
    // 当第二帧来了之后，创建了一些新的体素，这些体素肯定还没被估计，直到voxel中的点达到一个阈值
    // 才开始估计
    if (!v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) {
        math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3d& pt) { return pt; });
        v.info_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse();
        v.ndt_estimated_ = true;
        // 清理是为了存储新来的点
        v.pts_.clear();
    } else if (v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) {
        // 这里比如第一帧所有的体素都被估计了，然后第二帧，第三帧。。。持续到来，于是当voxel的点到达阈值
        // 重新估计协方差和均值
        Vec3d cur_mu, new_mu;
        Mat3d cur_var, new_var;
        math::ComputeMeanAndCov(v.pts_, cur_mu, cur_var, [this](const Vec3d& p) { return p; });
        math::UpdateMeanAndCov(v.num_pts_, v.pts_.size(), v.mu_, v.sigma_, cur_mu, cur_var, new_mu, new_var);
        v.mu_ = new_mu;
        v.sigma_ = new_var;
        v.num_pts_ += v.pts_.size();
        v.pts_.clear();

        // check info
        Eigen::JacobiSVD svd(v.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3d lambda = svd.singularValues();
        if (lambda[1] < lambda[0] * 1e-3) {
            lambda[1] = lambda[0] * 1e-3;
        }

        if (lambda[2] < lambda[0] * 1e-3) {
            lambda[2] = lambda[0] * 1e-3;
        }

        Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        v.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
}

/// 使用gauss-newton方法进行ndt配准
bool IncNdt3d::AlignNdt(SE3& init_pose) {
    LOG(INFO) << "aligning with inc ndt, pts: " << source_->size() << ", grids: " << grids_.size();
    assert(grids_.empty() == false);
    SE3 pose = init_pose;

    // 对点的索引，预先生成
    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6) {
        num_residual_per_point = 7;
    }

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // 我们来写一些并发代码
    int total_size = index.size() * num_residual_per_point;
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        std::vector<bool> effect_pts(total_size, false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vec3d> errors(total_size);
        std::vector<Mat3d> infos(total_size);
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto point_origin = lidar_utils::ToVec3d(source_->points[idx]);
            auto point_transed = pose * point_origin;
            // 计算栅格
            Vec3i key = lidar_utils::CastToInt(Vec3d(point_transed * options_.inv_voxel_size_));
            // 遍历周围的体素
            for (int i = 0; i < nearby_grids_.size(); ++i) {
                Vec3i real_key = key + nearby_grids_[i];
                auto iter = grids_.find(real_key);
                int real_idx = idx * num_residual_per_point + i;
                // 找到了体素并且已经估计过均值和协方差
                if (iter != grids_.end() && iter->second->second.ndt_estimated_) {
                    auto voxel = iter->second->second;
                    Vec3d e = point_transed - voxel.mu_;
                    // check chi2 th
                    double res = e.transpose() * voxel.info_ * e;
                    if (std::isnan(res) || res > options_.res_outlier_th_) {
                        effect_pts[real_idx] = false;
                        continue;
                    }
                    // 同ndt的残差构建一样
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(point_origin);
                    J.block<3, 3>(0, 3) = Mat3d::Identity();

                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = voxel.info_;
                    effect_pts[real_idx] = true;
                } else {
                    effect_pts[real_idx] = false;
                }
            }
        });
        // 计算H,b,dx
        // 累加Hessian和error,计算dx
        double total_res = 0;

        int effective_num = 0;

        Mat6d H = Mat6d::Zero();
        Vec6d err = Vec6d::Zero();

        for (int idx = 0; idx < effect_pts.size(); ++idx) {
            if (!effect_pts[idx]) {
                continue;
            }

            total_res += errors[idx].transpose() * infos[idx] * errors[idx];
            effective_num++;

            H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
            err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
        }

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            init_pose = pose;
            return false;
        }

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm()
                  << ", dx: " << dx.transpose();

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }

    init_pose = pose;
    return true;
}

void IncNdt3d::ComputeResidualAndJacobians(const SE3& input_pose, Mat18d& HTVH, Vec18d& HTVr) {
    assert(grids_.empty() == false);
    SE3 pose = input_pose;
    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6) {
        num_residual_per_point = 7;
    }
    std::vector<int> index(source_->points.size());
    std::iota(index.begin(), index.end(), 0);
    int total_size = index.size() * num_residual_per_point;

    // 注意这里的jacobian维数不一样了
    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 3, 18>> jacobians(total_size);
    std::vector<Vec3d> errors(total_size);
    std::vector<Mat3d> infos(total_size);

    // gauss-newton 迭代
    // 最近邻，可以并发
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
        auto point_origin = lidar_utils::ToVec3d(source_->points[idx]);
        auto point_transed = pose * point_origin;
        // 计算坐标
        Vec3i key = lidar_utils::CastToInt(Vec3d(point_transed * options_.inv_voxel_size_));
        // 找邻近的体素
        for (int i = 0; i < nearby_grids_.size(); ++i) {
            Vec3i real_key = key + nearby_grids_[i];
            auto it = grids_.find(real_key);
            int real_idx = idx * num_residual_per_point + i;
            // 检查体素中高斯是否已经估计过
            if (it != grids_.end() && it->second->second.ndt_estimated_) {
                auto& voxel = it->second->second;
                Vec3d e = point_transed - voxel.mu_;
                // check chi2 th
                double res = e.transpose() * voxel.info_ * e;
                if (std::isnan(res) || res > options_.res_outlier_th_) {
                    effect_pts[real_idx] = false;
                    continue;
                }

                // build residual
                Eigen::Matrix<double, 3, 18> J;
                J.setZero();
                J.block<3, 3>(0, 0) = Mat3d::Identity();                              // 对p
                J.block<3, 3>(0, 6) = -pose.so3().matrix() * SO3::hat(point_origin);  // 对R
                jacobians[real_idx] = J;
                errors[real_idx] = e;
                infos[real_idx] = voxel.info_;
                effect_pts[real_idx] = true;
            } else {
                effect_pts[real_idx] = false;
            }
        }
    });

    // 累加Hessian和error,计算dx
    double total_res = 0;
    int effective_num = 0;

    HTVH.setZero();
    HTVr.setZero();

    const double info_ratio = 0.01;  // 每个点反馈的info因子

    for (int idx = 0; idx < effect_pts.size(); ++idx) {
        if (!effect_pts[idx]) {
            continue;
        }

        total_res += errors[idx].transpose() * infos[idx] * errors[idx];
        effective_num++;

        HTVH += jacobians[idx].transpose() * infos[idx] * jacobians[idx] * info_ratio;
        HTVr += -jacobians[idx].transpose() * infos[idx] * errors[idx] * info_ratio;
    }

    LOG(INFO) << "effective: " << effective_num;
}
}  // namespace slam_learn::ndt_inc
