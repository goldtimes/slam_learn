#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/rosbag_io.hh"
#include "common/sensors/point_type.hh"
#include "lidar_slam/ch7/loam-like/feature_extraction.hh"

#include "common/lidar_utils.hh"
#include "common/timer/timer.hh"

/// 这里需要vlp16的数据，用wxb的
DEFINE_string(bag_path, "/home/kilox/slam_auto_driving/wxb/test1.bag", "path to wxb bag");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 测试角点和平面点的提取
    slam_learn::loam::FeatureExtraction feature_extraction;

    system("rm -rf ./data/ch7/*.pcd");

    slam_learn::rosbag_io::RosbagIO bag_io(fLS::FLAGS_bag_path);
    using namespace slam_learn;
    bag_io
        .AddVelodyneHandle("/velodyne_packets_1",
                           [&](FullCloudPtr cloud) -> bool {
                               CloudPtr pcd_corner(new PointCloudXYZI), pcd_surf(new PointCloudXYZI);
                               timer::Timer::Evaluate(
                                   [&]() { feature_extraction.Extract(cloud, pcd_corner, pcd_surf); },
                                   "Feature Extraction");
                               LOG(INFO) << "original pts:" << cloud->size() << ", corners: " << pcd_corner->size()
                                         << ", surf: " << pcd_surf->size();
                               lidar_utils::SaveCloudToFile("./data/ch7/corner.pcd", *pcd_corner);
                               lidar_utils::SaveCloudToFile("./data/ch7/surf.pcd", *pcd_surf);
                               return true;
                           })
        .Go();

    timer::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
