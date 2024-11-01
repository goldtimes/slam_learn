#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "lidar_slam/ch5/bfnn.hh"
#include "lidar_slam/ch5/gridnn.hpp"
#include "lidar_slam/ch5/kdtree.hh"
// #include "ch5/kdtree.h"
// #include "ch5/octo_tree.h"
// #include "common/point_cloud_utils.h"
#include "common/lidar_utils.hh"
#include "common/sensors/point_type.hh"
// #include "common/sys_utils.h"
#include <chrono>
#include "common/math_utils.hh"

DEFINE_string(first_scan_path, "./data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "./data/ch5/second.pcd", "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");

template <typename F>
void evaluate_and_call(F&& func, std::string func_name, double times) {
    double total_time = 0;
    for (int i = 0; i < times; ++i) {
        auto t1 = std::chrono::high_resolution_clock::now();
        func();
        auto t2 = std::chrono::high_resolution_clock::now();
        total_time += std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    }

    LOG(INFO) << "方法 " << func_name << " 平均调用时间/次数: " << total_time / times << "/" << times << " 毫秒.";
}

TEST(CH5_TEST, BFNN) {
    using namespace slam_learn;
    CloudPtr first(new PointCloudXYZI), second(new PointCloudXYZI);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    lidar_utils::VoxelGrid(first);
    lidar_utils::VoxelGrid(second);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    // 评价单线程和多线程版本的暴力匹配
    evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            bfnn::bfnn_cloud(first, second, matches);
            LOG(INFO) << "暴力匹配(单线程)matches size:" << matches.size();
        },
        "暴力匹配（单线程）", 5);
    evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            bfnn::bfnn_cloud_mt(first, second, matches);
            LOG(INFO) << "暴力匹配(多线程)matches size:" << matches.size();
        },
        "暴力匹配（多线程）", 5);

    SUCCEED();
}

/**
 * @brief 评价最邻近的正确性
 */
void EvaluateMatches(const std::vector<std::pair<size_t, size_t>>& truth,
                     const std::vector<std::pair<size_t, size_t>>& esti) {
    using namespace slam_learn;
    int fp = 0;  // 假阳性 算法找到最邻近，但是不是最邻近 esti中的值在truth集合中不存在
    int fn = 0;  // 假阴性 算法没找到最邻近，但是存在最邻近 truth集合中的值，在esti中不存在
    LOG(INFO) << "truth: " << truth.size() << ", esti: " << esti.size();

    // 检查某个匹配是否在容器中
    auto exist = [](const std::pair<size_t, size_t> query_data,
                    const std::vector<std::pair<size_t, size_t>>& target) -> bool {
        auto iter = std::find(target.begin(), target.end(), query_data);
        return iter != target.end();
    };

    int effective_esti = 0;
    // 遍历gridnn 查找到的匹配值
    for (const auto& d : esti) {
        // 存在最邻近的匹配
        if (d.first != math::kINVALID_ID && d.second != math::kINVALID_ID) {
            effective_esti++;
            // 验证这个是不是假阳性
            if (!exist(d, truth)) {
                fp++;
            }
        }
    }
    // 验证是不是假阴性
    for (const auto& d : truth) {
        if (!exist(d, esti)) {
            fn++;
        }
    }
    float precision = 1.0 - float(fp) / effective_esti;
    float recall = 1.0 - float(fn) / truth.size();
    LOG(INFO) << "precision: " << precision << ", recall: " << recall << ", fp: " << fp << ", fn: " << fn;
}

TEST(CH5_TEST, GRID_NN) {
    using namespace slam_learn;
    CloudPtr first(new PointCloudXYZI), second(new PointCloudXYZI);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    lidar_utils::VoxelGrid(first);
    lidar_utils::VoxelGrid(second);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();
    std::vector<std::pair<size_t, size_t>> truth_matches;
    // 计算一遍真值
    bfnn::bfnn_cloud(first, second, truth_matches);
    // 对比不同种类的grid
    GridNN<2> grid0(0.1, GridNN<2>::NearbyType::CENTER);
    GridNN<2> grid4(0.1, GridNN<2>::NearbyType::NEARBY4);
    GridNN<2> grid8(0.1, GridNN<2>::NearbyType::NEARBY8);
    GridNN<3> grid3(0.1, GridNN<3>::NearbyType::NEARBY6);
    // 设置点云
    grid0.SetPointCloud(first);
    grid4.SetPointCloud(first);
    grid8.SetPointCloud(first);
    grid3.SetPointCloud(first);

    // 评价各种版本的Grid NN
    // sorry没有C17的template lambda... 下面必须写的啰嗦一些
    LOG(INFO) << "===================";
    std::vector<std::pair<size_t, size_t>> matches;
    evaluate_and_call([&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloud(first, second, matches); },
                      "Grid0 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call(
        [&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloudMT(first, second, matches); },
        "Grid0 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloud(first, second, matches); },
                      "Grid4 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call(
        [&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloudMT(first, second, matches); },
        "Grid4 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloud(first, second, matches); },
                      "Grid8 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call(
        [&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloudMT(first, second, matches); },
        "Grid8 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloud(first, second, matches); },
                      "Grid 3D 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call(
        [&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloudMT(first, second, matches); },
        "Grid 3D 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    SUCCEED();
}

TEST(CH5_TEST, KDTREE_BASICS) {
    using namespace slam_learn;
    CloudPtr cloud(new PointCloudXYZI);
    PointXYZI p1, p2, p3, p4;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;

    p2.x = 1;
    p2.y = 0;
    p2.z = 0;

    p3.x = 0;
    p3.y = 1;
    p3.z = 0;

    p4.x = 1;
    p4.y = 1;
    p4.z = 0;

    cloud->points.push_back(p1);
    cloud->points.push_back(p2);
    cloud->points.push_back(p3);
    cloud->points.push_back(p4);

    kdtree::KdTree kdtree;
    kdtree.BuildTree(cloud);
    kdtree.PrintAll();

    SUCCEED();
}

TEST(CH5_TEST, KDTREE_KNN) {
    using namespace slam_learn;
    CloudPtr first(new PointCloudXYZI), second(new PointCloudXYZI);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    lidar_utils::VoxelGrid(first);
    lidar_utils::VoxelGrid(second);

    kdtree::KdTree kdtree;
    evaluate_and_call([&first, &kdtree]() { kdtree.BuildTree(first); }, "Kd Tree build", 1);

    kdtree.SetEnableANN(true, FLAGS_ANN_alpha);

    LOG(INFO) << "Kd tree leaves: " << kdtree.size() << ", points: " << first->size();

    // 比较 bfnn
    std::vector<std::pair<size_t, size_t>> true_matches;
    bfnn::bfnn_cloud_mt_k(first, second, true_matches);

    // 对第2个点云执行knn
    std::vector<std::pair<size_t, size_t>> matches;
    evaluate_and_call([&first, &second, &kdtree, &matches]() { kdtree.GetClosestPointMT(second, matches, 5); },
                      "Kd Tree 5NN 多线程", 1);
    EvaluateMatches(true_matches, matches);

    LOG(INFO) << "building kdtree pcl";
    // 对比PCL
    pcl::search::KdTree<PointXYZI> kdtree_pcl;
    evaluate_and_call([&first, &kdtree_pcl]() { kdtree_pcl.setInputCloud(first); }, "Kd Tree build", 1);

    LOG(INFO) << "searching pcl";
    matches.clear();
    std::vector<int> search_indices(second->size());
    for (int i = 0; i < second->points.size(); i++) {
        search_indices[i] = i;
    }

    std::vector<std::vector<int>> result_index;
    std::vector<std::vector<float>> result_distance;
    evaluate_and_call([&]() { kdtree_pcl.nearestKSearch(*second, search_indices, 5, result_index, result_distance); },
                      "Kd Tree 5NN in PCL", 1);
    for (int i = 0; i < second->points.size(); i++) {
        for (int j = 0; j < result_index[i].size(); ++j) {
            int m = result_index[i][j];
            double d = result_distance[i][j];
            matches.push_back({m, i});
        }
    }
    EvaluateMatches(true_matches, matches);

    LOG(INFO) << "done.";

    SUCCEED();
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);
    return RUN_ALL_TESTS();
}