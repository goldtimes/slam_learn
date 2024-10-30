#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "./data/ch5/map_example.pcd", "点云文件路径");
DEFINE_double(image_resolution, 0.1, "俯视图分辨率");
DEFINE_double(min_z, 0.2, "俯视图最低高度");
DEFINE_double(max_z, 2.5, "俯视图最高高度");

/// 本节演示如何将一个点云转换为俯视图像
void GenerateBEVImage(PointCloudType::Ptr cloud) {
    // 获取点云的最大最小范围
    auto minmax_x =
        std::minmax_element(cloud->points.begin(), cloud->points.end(),
                            [&](const PointType& point1, const PointType& point2) { return point1.x < point2.x; });
    auto minmax_y =
        std::minmax_element(cloud->points.begin(), cloud->points.end(),
                            [&](const PointType& point1, const PointType& point2) { return point1.y < point2.y; });
    // 确定图像的尺寸
    double min_x = minmax_x.first->x;
    double max_x = minmax_x.second->x;
    double min_y = minmax_y.first->y;
    double max_y = minmax_y.second->y;
    double res_inv = 1 / FLAGS_image_resolution;

    const int image_rows = int((max_y - min_y) * res_inv);
    const int image_cols = int((max_x - min_x) * res_inv);
    // 将点云中心作为原点，然后以图像为中心作为原点
    float x_center = 0.5 * (max_x + min_x);
    float y_center = 0.5 * (max_y + min_y);
    float x_center_image = image_cols / 2;
    float y_center_image = image_rows / 2;

    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& pt : cloud->points) {
        int x = int((pt.x - x_center) * res_inv + x_center_image);
        int y = int((pt.y - y_center) * res_inv + y_center_image);
        if (x < 0 || x >= image_cols || y < 0 || y >= image_rows || pt.z < FLAGS_min_z || pt.z > FLAGS_max_z) {
            continue;
        }

        image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);
    }

    cv::imwrite("./bev.png", image);
    cv::namedWindow("bev", cv::WINDOW_AUTOSIZE);
    cv::imshow("bev", image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_pcd_path.empty()) {
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }

    // 读取点云
    PointCloudType::Ptr cloud(new PointCloudType);
    pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);
    if (cloud->empty()) {
        LOG(ERROR) << "cannot load cloud file";
        return -1;
    }

    LOG(INFO) << "cloud points: " << cloud->size();
    GenerateBEVImage(cloud);
}