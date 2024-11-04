#include <pcl/search/kdtree.h>
#include <deque>
#include <memory>
#include "common/eigen_types.hh"
#include "common/sensors/point_type.hh"
#include "lidar_slam/ch5/kdtree.hh"
#include "lidar_slam/ch7/icp_3d.hh"
#include "lidar_slam/ch7/loam-like/feature_extraction.hh"
#include "tools/pcl_map_viewer.hh"

namespace slam_learn::loam {
// 前置声明
class FeatureExtraction;
class LoamLikeOdom {
   public:
    struct Options {
        Options() {
        }

        int min_edge_pts_ = 20;               // 最小边缘点数
        int min_surf_pts_ = 20;               // 最小平面点数
        double kf_distance_ = 1.0;            // 关键帧距离
        double kf_angle_deg_ = 15;            // 旋转角度
        int num_kfs_in_local_map_ = 30;       // 局部地图含有多少个关键帧
        bool display_realtime_cloud_ = true;  // 是否显示实时点云

        // ICP 参数
        int max_iteration_ = 5;             // 最大迭代次数
        double max_plane_distance_ = 0.05;  // 平面最近邻查找时阈值
        double max_line_distance_ = 0.5;    // 点线最近邻查找时阈值
        int min_effective_pts_ = 10;        // 最近邻点数阈值
        double eps_ = 1e-3;                 // 收敛判定条件

        bool use_edge_points_ = true;  // 是否使用边缘点
        bool use_surf_points_ = true;  // 是否使用平面点
        bool use_pcl_kdtree_ = true;
    };

   public:
    explicit LoamLikeOdom(Options options = Options());

    /**
     * 往里程计中添加一个点云，内部会分为角点和平面点
     * @param full_cloud
     */
    void ProcessPointCloud(FullCloudPtr full_cloud);

    void SaveMap(const std::string& path);

   private:
    /// 与局部地图进行配准
    SE3 AlignWithLocalMap(CloudPtr edge, CloudPtr surf);

    /// 判定是否为关键帧
    bool IsKeyframe(const SE3& current_pose);

   private:
    Options options_;

    int cnt_frame_ = 0;
    int last_kf_id_ = 0;
    // pcl_kdtree;
    pcl::search::KdTree<PointXYZI> pcl_kdtree_edge_;
    pcl::search::KdTree<PointXYZI> pcl_kdtree_surf_;
    //    tree
    kdtree::KdTree kdtree_edge_;
    kdtree::KdTree kdtree_surf_;
    // cloud
    CloudPtr submap_edge_;
    CloudPtr submap_surf_;
    CloudPtr global_map_;
    // keyframe
    std::deque<CloudPtr> edges_keyframes_buffer_;
    std::deque<CloudPtr> surf_keyframes_buffer_;
    // feature
    std::shared_ptr<FeatureExtraction> feature_extraction_ = nullptr;
    // viewer
    std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
    // 所有估计出来的pose，用于记录轨迹和预测下一个帧
    std::vector<SE3> estimated_poses_;
    // 上一关键帧的位姿
    SE3 last_kf_pose_;
};
}  // namespace slam_learn::loam