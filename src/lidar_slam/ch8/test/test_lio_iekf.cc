#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/rosbag_io.hh"
#include "common/timer/timer.hh"
#include "lidar_slam/ch8/lio-iekf/lio_iekf.hh"

DEFINE_string(bag_path, "/home/kilox/slam_auto_driving/nclt/20120511.bag", "path to rosbag");
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "./config/velodyne_nclt.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace slam_learn;
    using namespace rosbag_io;

    RosbagIO bag_io(fLS::FLAGS_bag_path, Str2DatasetType(FLAGS_dataset_type));

    ieskf_lio::LioIEKF lio;
    lio.Init(FLAGS_config);

    bag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            timer::Timer::Evaluate([&]() { lio.PCLCallBack(cloud); }, "IEKF lio");
            return true;
        })
        // .AddLivoxHandle([&](const livox_ros_driver::CustomMsg::ConstPtr& msg) -> bool {
        //     timer::Timer::Evaluate([&]() { lio.LivoxPCLCallBack(msg); }, "IEKF lio");
        //     return true;
        // })
        .AddImuHandle([&](IMUPtr imu) {
            lio.IMUCallBack(imu);
            return true;
        })
        .Go();

    lio.Finish();
    timer::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}