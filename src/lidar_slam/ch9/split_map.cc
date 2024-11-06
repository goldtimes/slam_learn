#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "common/compare_func.hh"
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/sensors/point_type.hh"
#include "lidar_slam/ch9/keyframe.hh"

DEFINE_string(map_path, "./data/ch9/", "导出数据的目录");
DEFINE_double(voxel_size, 0.1, "导出地图分辨率");

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
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data;  // 以网格ID为索引的地图数据
    pcl::VoxelGrid<PointXYZI> voxel_grid_filter;
    float resolution = FLAGS_voxel_size;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    // 遍历所有keyframe
    for (auto& kfp : keyframes) {
        // 加载点云
        auto kf = kfp.second;
        kf->LoadScan("./data/ch9/");

        CloudPtr cloud_trans(new PointCloudXYZI);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, kf->opti_pose_2_.matrix());

        // 降采样
        CloudPtr kf_cloud_voxeled(new PointCloudXYZI);
        voxel_grid_filter.setInputCloud(cloud_trans);
        voxel_grid_filter.filter(*kf_cloud_voxeled);

        LOG(INFO) << "building kf " << kf->id_ << " in " << keyframes.size();
        // 这里我们以100米为边长，对点云进行分割，切以地图的中心为原点
        for (const auto& pt : kf_cloud_voxeled->points) {
            int gx = floor(pt.x - 50.0) / 100;
            int gy = floor(pt.y - 50.0) / 100;
            Vec2i key(gx, gy);
            auto iter = map_data.find(key);
            if (iter == map_data.end()) {
                CloudPtr cloud(new PointCloudXYZI);
                cloud->points.emplace_back(pt);
                cloud->is_dense = false;
                cloud->height = 1;
                map_data.emplace(key, cloud);
            } else {
                map_data[key]->emplace_back(pt);
            }
        }
    }

    // 存储点云和索引文件
    LOG(INFO) << "saving maps, grids: " << map_data.size();
    std::system("mkdir -p ./data/ch9/map_data/");
    std::system("rm -rf ./data/ch9/map_data/*");  // 清理一下文件夹
    std::ofstream fout("./data/ch9/map_data/map_index.txt");
    for (auto& dp : map_data) {
        fout << dp.first[0] << " " << dp.first[1] << std::endl;
        dp.second->width = dp.second->size();
        lidar_utils::VoxelGrid(dp.second, 0.1);
        lidar_utils::SaveCloudToFile(
            "./data/ch9/map_data/" + std::to_string(dp.first[0]) + "_" + std::to_string(dp.first[1]) + ".pcd",
            *dp.second);
    }
    fout.close();

    LOG(INFO) << "done.";
    return 0;
}