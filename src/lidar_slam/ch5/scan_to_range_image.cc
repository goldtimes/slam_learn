#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "./data/ch5/scan_example.pcd", "点云文件路径");
DEFINE_double(azimuth_resolution_deg, 0.3, "方位角分辨率（度）");
DEFINE_int32(elevation_rows, 16, "俯仰角对应的行数");
DEFINE_double(elevation_range, 15.0, "俯仰角范围");  // VLP-16 上下各15度范围
DEFINE_double(lidar_height, 1.128, "雷达安装高度");

/// 本节演示如何将一个点云转换为俯视图像
void GenerateRangeImage(PointCloudType::Ptr cloud) {
    // 这里需要计算相片的分辨率 16 * (360 / 0.3)
    int image_cols = int(360 / FLAGS_azimuth_resolution_deg);  // 水平为360度，按分辨率切分即可
    int image_rows = FLAGS_elevation_rows;                     // 图像行数固定
    LOG(INFO) << "range image: " << image_rows << "x" << image_cols;

    // 我们生成一个HSV图像以更好地显示图像
    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(0, 0, 0));
    // 垂直分辨率
    double ele_resolution = FLAGS_elevation_range * 2 / FLAGS_elevation_rows;  // elevation分辨率
    // 计算方位角
    for (const auto& point : cloud->points) {
        double azimuth = std::atan2(point.y, point.x) * 180 / M_PI;
        double range = std::sqrt(point.x * point.x + point.y * point.y +
                                 (point.z - FLAGS_lidar_height) * (point.z - FLAGS_lidar_height));
        double elevation = std::asin((point.z - FLAGS_lidar_height) / range) * 180 / M_PI;

        // keep in 0~360
        if (azimuth < 0) {
            azimuth += 360;
        }

        int x = int(azimuth / FLAGS_azimuth_resolution_deg);                      // 行
        int y = int((elevation + FLAGS_elevation_range) / ele_resolution + 0.5);  // 列

        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(range / 100 * 255.0), 255, 127);
        }
    }
    cv::Mat image_flipped;
    cv::flip(image, image_flipped, 0);

    // hsv to rgb
    cv::Mat image_rgb;
    cv::cvtColor(image_flipped, image_rgb, cv::COLOR_HSV2BGR);
    cv::imwrite("./range_image.png", image_rgb);

    cv::imwrite("./range_mat.png", image_rgb);
    cv::namedWindow("range_image", cv::WINDOW_AUTOSIZE);
    cv::imshow("range_image", image_rgb);
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
    GenerateRangeImage(cloud);
}