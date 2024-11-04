#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/rosbag_io.hh"
#include "common/timer/timer.hh"
#include "lidar_slam/ch7/loam-like/loam_like_odom.hh"

DEFINE_string(bag_path, "/home/kilox/slam_auto_driving/wxb/test1.bag", "path to wxb bag");
DEFINE_string(topic, "/velodyne_packets_1", "topic of lidar packets");
DEFINE_bool(use_pcl_kdtree, true, "use pcl kdtree");
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace slam_learn;
    // 测试loam-like odometry的表现
    loam::LoamLikeOdom::Options options;
    options.use_pcl_kdtree_ = FLAGS_use_pcl_kdtree;
    options.display_realtime_cloud_ = FLAGS_display_map;
    loam::LoamLikeOdom lo(options);

    LOG(INFO) << "using topic: " << FLAGS_topic;
    rosbag_io::RosbagIO bag_io(fLS::FLAGS_bag_path);
    bag_io
        .AddVelodyneHandle(FLAGS_topic,
                           [&](FullCloudPtr cloud) -> bool {
                               timer::Timer::Evaluate([&]() { lo.ProcessPointCloud(cloud); }, "Loam-like odom");
                               return true;
                           })
        .Go();

    lo.SaveMap("./data/ch7/loam_map.pcd");

    timer::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}