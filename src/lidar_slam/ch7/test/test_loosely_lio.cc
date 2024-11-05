#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/rosbag_io.hh"
#include "common/timer/timer.hh"
#include "lidar_slam/ch7/loosely_coupled_lio/loosely_lio.hh"

DEFINE_string(bag_path, "/home/kilox/slam_auto_driving/ulhk/test3.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "./config/velodyne_ulhk.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    using namespace slam_learn;
    using namespace rosbag_io;
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    RosbagIO rosbag_io(fLS::FLAGS_bag_path, Str2DatasetType(FLAGS_dataset_type));

    loosely::LooselyLIO::Options options;
    options.with_ui_ = FLAGS_display_map;
    loosely::LooselyLIO lm(options);
    lm.Init(FLAGS_config);

    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            timer::Timer::Evaluate([&]() { lm.PCLCallBack(cloud); }, "loosely lio");
            return true;
        })
        .AddLivoxHandle([&](const livox_ros_driver::CustomMsg::ConstPtr& msg) -> bool {
            timer::Timer::Evaluate([&]() { lm.LivoxPCLCallBack(msg); }, "loosely lio");
            return true;
        })
        .AddImuHandle([&](IMUPtr imu) {
            lm.IMUCallBack(imu);
            return true;
        })
        .Go();

    lm.Finish();
    timer::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}