#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/dataset_type.hh"
#include "common/lidar_utils.hh"
#include "common/rosbag_io.hh"
#include "common/timer/timer.hh"
#include "lidar_slam/ch7/direct_ndt_lo.hh"
#include "lidar_slam/ch7/ndt_3d.hh"

/// 本程序以ULHK数据集为例
/// 测试以NDT为主的Lidar Odometry
/// 若使用PCL NDT的话，会重新建立NDT树
DEFINE_string(bag_path, "/home/kilox/slam_auto_driving/ulhk/test2.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB_3D");  // 数据集类型
DEFINE_bool(use_pcl_ndt, false, "use pcl ndt to align?");
DEFINE_bool(use_ndt_nearby_6, false, "use ndt nearby 6?");
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    using namespace slam_learn;
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    rosbag_io::RosbagIO bag_io(fLS::FLAGS_bag_path, Str2DatasetType(FLAGS_dataset_type));

    ndt_lo::DirectNDTLO::Options options;
    options.use_pcl_ndt = fLB::FLAGS_use_pcl_ndt;
    options.ndt3d_options_.nearby_type_ =
        FLAGS_use_ndt_nearby_6 ? ndt_3d::Ndt3d::NearbyType::NEARBY6 : ndt_3d::Ndt3d::NearbyType::CENTER;
    options.display_realtime_cloud_ = FLAGS_display_map;
    ndt_lo::DirectNDTLO ndt_lo(options);

    bag_io
        .AddAutoPointCloudHandle([&ndt_lo](sensor_msgs::PointCloud2::Ptr msg) -> bool {
            timer::Timer::Evaluate(
                [&]() {
                    SE3 pose;

                    ndt_lo.AddCloud(lidar_utils::VoxelCloud(lidar_utils::PointCloud2ToCloudPtr(msg), 0.1), pose);
                },
                "NDT registration");
            return true;
        })
        .Go();

    if (FLAGS_display_map) {
        // 把地图存下来
        ndt_lo.SaveMap("./data/ch7/map.pcd");
    }

    timer::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}