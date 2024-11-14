#include <glog/logging.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <numeric>
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/sensors/point_type.hh"

namespace slam_learn::icp_3d {
class Icp3d {
   public:
    struct Options {
        int max_iteration_ = 20;                // 最大迭代次数
        double max_nn_distance_ = 1.0;          // 点到点最近邻查找时阈值
        double max_plane_distance_ = 0.05;      // 平面最近邻查找时阈值
        double max_line_distance_ = 0.5;        // 点线最近邻查找时阈值
        int min_effective_pts_ = 10;            // 最近邻点数阈值
        double eps_ = 1e-2;                     // 收敛判定条件
        bool use_initial_translation_ = false;  // 是否使用初始位姿中的平移估计
    };

   public:
    Icp3d() {
    }
    Icp3d(Options options) : options_(options) {
    }

    void SetTarget(CloudPtr target_cloud) {
        target_ = target_cloud;
        BuildTargetTree();
        // 计算点云中心
        target_center_ = std::accumulate(target_cloud->points.begin(), target_cloud->points.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& sum, const PointXYZI& point) -> Vec3d {
                                             return sum + lidar_utils::ToVec3d(point);
                                         }) /
                         target_cloud->size();
        LOG(INFO) << "target center: " << target_center_.transpose();
    }

    void SetSource(CloudPtr source_cloud) {
        source_ = source_cloud;
        source_center_ =
            std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                            [](const Vec3d& c, const PointXYZI& pt) -> Vec3d { return c + lidar_utils::ToVec3d(pt); }) /
            source_->size();
        LOG(INFO) << "source center: " << source_center_.transpose();
    }

    void SetGroundTruth(const SE3& gt_pose) {
        gt_pose_ = gt_pose;
        gt_set_ = true;
    }

    /// 使用gauss-newton方法进行配准, 点到点
    bool AlignP2P(SE3& init_pose);

    /// 基于gauss-newton的点线ICP
    bool AlignP2Line(SE3& init_pose);

    /// 基于gauss-newton的点面ICP
    bool AlignP2Plane(SE3& init_pose);

    /// 基于LM算法的点面ICP
    bool AlignP2PlaneLM(SE3& init_pose);

   private:
    void BuildTargetTree();

   private:
    pcl::search::KdTree<PointXYZI> kdtree_;

    Options options_;
    CloudPtr target_ = nullptr;
    CloudPtr source_ = nullptr;

    Vec3d target_center_ = Vec3d::Zero();
    Vec3d source_center_ = Vec3d::Zero();

    bool gt_set_ = false;
    SE3 gt_pose_;
};
}  // namespace slam_learn::icp_3d