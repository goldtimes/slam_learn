#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include "common/rosbag_io.hh"
#include "lidar_slam/ch6/lidar_2d_utils.hh"

DEFINE_string(bag_path, "./dataset/sad/2dmapping/test_2d_lidar.bag", "数据包路径");

int main(int argc, char** argv) {
    using namespace slam_learn;
    google::InitGoogleLogging(argv[0]);
    fLI::FLAGS_stderrthreshold = google::INFO;
    fLB::FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    slam_learn::rosbag_io::RosbagIO io(FLAGS_bag_path);
    io.AddScan2DHandle("/pavo_scan_bottom", [](sensor_msgs::LaserScan::Ptr msg) -> bool {
          cv::Mat image;
          slam_learn::Visualize2DScan(msg, SE2(), image, Vec3b(255, 0, 0));
          cv::imshow("scan", image);
          cv::waitKey(20);
          return true;
      }).Go();
}