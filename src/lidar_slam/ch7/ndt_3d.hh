#pragma once

#include <unordered_map>
#include <vector>
#include "common/compare_func.hh"
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/sensors/point_type.hh"

namespace slam_learn::ndt_3d {
class Ndt3d {
   public:
    enum class NearbyType {
        CENTER,   // 只考虑中心
        NEARBY6,  // 上下左右前后
    };
    using KeyType = Eigen::Matrix<int, 3, 1>;

    struct Options {
        int max_iteration_ = 20;        // 最大迭代次数
        double voxel_size_ = 1.0;       // 体素大小
        double inv_voxel_size_ = 1.0;   //
        int min_effective_pts_ = 10;    // 最近邻点数阈值
        int min_pts_in_voxel_ = 3;      // 每个栅格中最小点数
        double eps_ = 1e-2;             // 收敛判定条件
        double res_outlier_th_ = 20.0;  // 异常值拒绝阈值
        bool remove_centroid_ = false;  // 是否计算两个点云中心并移除中心？

        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };

    // 建立体素中的点云均值和方差
    struct VoxelData {
        VoxelData() {
        }
        VoxelData(size_t id) {
            idx_.emplace_back(id);
        }
        // 存放点云的索引
        std::vector<size_t> idx_;
        // 均值
        Vec3d mu_ = Vec3d::Zero();
        // 方差
        Mat3d sigma_ = Mat3d::Zero();
        // 协方差的尼
        Mat3d info_ = Mat3d::Zero();
    };

    Ndt3d() {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }

    Ndt3d(Options options) : options_(options) {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }

    /// 设置目标的Scan
    void SetTarget(CloudPtr target) {
        target_ = target;
        BuildVoxels();

        // 计算点云中心
        target_center_ =
            std::accumulate(target->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                            [](const Vec3d& c, const PointXYZI& pt) -> Vec3d { return c + lidar_utils::ToVec3d(pt); }) /
            target_->size();
    }

    /// 设置被配准的Scan
    void SetSource(CloudPtr source) {
        source_ = source;

        source_center_ =
            std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                            [](const Vec3d& c, const PointXYZI& pt) -> Vec3d { return c + lidar_utils::ToVec3d(pt); }) /
            source_->size();
    }

    void SetGtPose(const SE3& gt_pose) {
        gt_pose_ = gt_pose;
        gt_set_ = true;
    }

    /// 使用gauss-newton方法进行ndt配准
    bool AlignNdt(SE3& init_pose);

    /// 使用LM方法进行ndt配准
    bool AlignNdtLM(SE3& init_pose);

   private:
    void BuildVoxels();
    void GenerateNearbyGrids();

   private:
    CloudPtr target_ = nullptr;
    CloudPtr source_ = nullptr;

    Vec3d target_center_ = Vec3d::Zero();
    Vec3d source_center_ = Vec3d::Zero();

    SE3 gt_pose_;
    bool gt_set_ = false;
    Options options_;

    // 体素数据
    std::unordered_map<KeyType, VoxelData, hash_vec<3>> grids_;
    // 附近的体素
    std::vector<KeyType> nearby_grids_;
};
}  // namespace slam_learn::ndt_3d
