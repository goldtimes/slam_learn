#pragma once

#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/sensors/point_type.hh"

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>

namespace slam_learn::icp_2d {
/**
 * @brief scan-to-scan的高斯牛顿配准方式
 */
class Icp2d {
   public:
    using Point2d = pcl::PointXY;
    using Cloud2d = pcl::PointCloud<Point2d>;
    Icp2d() {
    }

    void SetTarget(Scan2d::Ptr target) {
        target_scan_ = target;
        BuildTargetKdTree();
    }
    // 高斯牛顿的配准方式
    bool AlignGaussNewton(SE2& init_pose);
    bool AlignGaussNewtonPoint2Line(SE2& init_pose);
    // LM算法的配准方式
    bool AlignLM(SE2& init_pose);
    bool AlignLMPoint2Line(SE2& init_pose);

   private:
    void BuildTargetKdTree();

   private:
    pcl::search::KdTree<Point2d> kdtree_;
    Cloud2d::Ptr target_cloud_;
    Scan2d::Ptr target_scan_;
    Scan2d::Ptr source_scan_;
};
}  // namespace slam_learn::icp_2d