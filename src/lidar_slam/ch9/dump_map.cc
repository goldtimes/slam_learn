#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "common/lidar_utils.hh"
#include "common/sensors/point_type.hh"
#include "lidar_slam/ch9/keyframe.hh"

DEFINE_double(voxel_size, 0.1, "导出地图分辨率");
DEFINE_string(pose_source, "lidar", "使用的pose来源:lidar/rtk/opti1/opti2");
DEFINE_string(dump_to, "./data/ch9/", "导出的目标路径");

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    fLI::FLAGS_stderrthreshold = google::INFO;
    fLB::FLAGS_colorlogtostderr = true;
    using namespace slam_learn;
    using namespace slam_learn::mapping;
    std::map<unsigned long, KeyFramePtr> keyframes;

    if (!LoadKeyFrames("./data/ch9/keyframes.txt", keyframes)) {
        LOG(ERROR) << "failed to load keyframes.txt";
        return -1;
    }

    if (keyframes.empty()) {
        LOG(INFO) << "keyframes are empty";
        return 0;
    }

    // dump kf cloud and merge
    LOG(INFO) << "merging";
    CloudPtr global_cloud(new PointCloudXYZI);

    pcl::VoxelGrid<PointXYZI> voxel_grid_filter;
    float resolution = FLAGS_voxel_size;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);
    int cnt = 0;

    for (auto& kfp : keyframes) {
        auto kf = kfp.second;
        SE3 pose;
        if (FLAGS_pose_source == "rtk") {
            pose = kf->rtk_pose_;
        } else if (FLAGS_pose_source == "lidar") {
            pose = kf->lidar_pose_;
        } else if (FLAGS_pose_source == "opti1") {
            pose = kf->opti_pose_1_;
        } else if (FLAGS_pose_source == "opti2") {
            pose = kf->opti_pose_2_;
        }

        kf->LoadScan("./data/ch9/");

        CloudPtr cloud_trans(new PointCloudXYZI);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix());

        // voxel size
        CloudPtr kf_cloud_voxeled(new PointCloudXYZI);
        voxel_grid_filter.setInputCloud(cloud_trans);
        voxel_grid_filter.filter(*kf_cloud_voxeled);

        *global_cloud += *kf_cloud_voxeled;
        kf->cloud_ = nullptr;

        LOG(INFO) << "merging " << cnt << " in " << keyframes.size() << ", pts: " << kf_cloud_voxeled->size()
                  << " global pts: " << global_cloud->size();
        cnt++;
    }

    if (!global_cloud->empty()) {
        lidar_utils::SaveCloudToFile(FLAGS_dump_to + "/map.pcd", *global_cloud);
    }

    LOG(INFO) << "done.";
    return 0;
}