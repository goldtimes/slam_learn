#pragma once
#include "common/compare_func.hh"
#include "common/eigen_types.hh"
#include "common/sensors/point_type.hh"

#include <list>
#include <unordered_map>
#include <utility>
#include <vector>

namespace slam_learn::ndt_inc {
class IncNdt3d {
   public:
    using KeyType = Eigen::Matrix<int, 3, 1>;

    enum class NearbyType {
        CENTER,
        NEARBY6,
    };

    struct Options {
        Options() {
        }
        int max_iteration_ = 4;        // 最大迭代次数
        double voxel_size_ = 1.0;      // 体素大小
        double inv_voxel_size_ = 1.0;  // 体素大小之逆
        int min_effective_pts_ = 10;   // 最近邻点数阈值
        int min_pts_in_voxel_ = 5;     // 每个栅格中最小点数
        int max_pts_in_voxel_ = 50;    // 每个栅格中最大点数
        double eps_ = 1e-3;            // 收敛判定条件
        double res_outlier_th_ = 5.0;  // 异常值拒绝阈值
        size_t capacity_ = 100000;     // 缓存的体素数量

        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };
    // 体素
    struct VoxelData {
        VoxelData() {
        }
        VoxelData(const Vec3d& pt) {
            pts_.emplace_back(pt);
            num_pts_ = 1;
        }

        void AddPoint(const Vec3d& pt) {
            pts_.emplace_back(pt);
            // 没被估计
            if (!ndt_estimated_) {
                num_pts_++;
            }
        }
        // 体素中存放的点,估计完均值和方差之后需要清空这个体素，等待接下来的点进入
        std::vector<Vec3d> pts_;
        // 均值
        Vec3d mu_ = Vec3d::Zero();
        // 协方差
        Mat3d sigma_ = Mat3d::Zero();
        Mat3d info_ = Mat3d::Zero();
        // ndt是否已经估计
        bool ndt_estimated_ = false;
        // voxel总的点数
        int num_pts_ = 0;
    };

   public:
    IncNdt3d() {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }

    IncNdt3d(Options options) : options_(options) {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }

    /// 获取一些统计信息
    int NumGrids() const {
        return grids_.size();
    }

    /// 在voxel里添加点云，
    void AddCloud(CloudPtr cloud_world);

    /// 设置被配准的Scan
    void SetSource(CloudPtr source) {
        source_ = source;
    }

    /// 使用gauss-newton方法进行ndt配准
    bool AlignNdt(SE3& init_pose);

   private:
    void GenerateNearbyGrids();

    /// 更新体素内部数据, 根据新加入的pts和历史的估计情况来确定自己的估计
    void UpdateVoxel(VoxelData& v);

   private:
    Options options_;
    CloudPtr source_ = nullptr;
    using KeyAndData = std::pair<KeyType, VoxelData>;
    // 存放体素和体素索引
    std::list<KeyAndData> data_;
    // 存放体素索引和对应的data;
    std::unordered_map<KeyType, std::list<KeyAndData>::iterator, hash_vec<3>> grids_;
    std::vector<KeyType> nearby_grids_;
    // 第一帧点云
    bool flag_first_scan_ = true;
};
}  // namespace slam_learn::ndt_inc