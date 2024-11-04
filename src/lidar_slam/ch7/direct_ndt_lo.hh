#pragma once
#include <pcl/registration/ndt.h>
#include <deque>
#include <vector>
#include "common/eigen_types.hh"
#include "common/sensors/point_type.hh"
#include "lidar_slam/ch7/ndt_3d.hh"
#include "tools/pcl_map_viewer.hh"
namespace slam_learn::ndt_lo {
/**
 * @brief 直接ndt里程计
 */
class DirectNDTLO {
   public:
    struct Options {
        Options() {
        }
        // keyframe
        double kf_distance = 0.5;
        double kf_angle = 30.0;
        int num_kfs = 30;
        bool use_pcl_ndt = true;
        bool display_realtime_cloud_ = true;
        ndt_3d::Ndt3d::Options ndt3d_options_;
    };

    DirectNDTLO(Options options = Options()) : options_(options) {
        if (options_.display_realtime_cloud_) {
            viewer_ = std::make_shared<PCLMapViewer>(0.5);
        }

        ndt_ = ndt_3d::Ndt3d(options_.ndt3d_options_);

        ndt_pcl_.setResolution(1.0);
        ndt_pcl_.setStepSize(0.1);
        ndt_pcl_.setTransformationEpsilon(0.01);
    }

    /**
     * 往LO中增加一个点云
     * @param scan  当前帧点云
     * @param pose 估计pose
     */
    void AddCloud(CloudPtr scan, SE3& pose);

    /// 存储地图(viewer里）
    void SaveMap(const std::string& map_path);

   private:
    /// 与local map进行配准
    SE3 AlignWithLocalMap(CloudPtr scan);

    /// 判定是否为关键帧
    bool IsKeyframe(const SE3& current_pose);

   private:
    Options options_;
    // submap
    CloudPtr submap_ = nullptr;
    // keyframes
    std::deque<CloudPtr> keyframes_buffer;
    // 轨迹
    std::vector<SE3> estimated_poses_;
    SE3 last_kf_pose;
    // pcl ndt
    pcl::NormalDistributionsTransform<PointXYZI, PointXYZI> ndt_pcl_;
    ndt_3d::Ndt3d ndt_;

    std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
};
}  // namespace slam_learn::ndt_lo