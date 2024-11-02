#include "lidar_slam/ch6/lidar_2d_utils.hh"
#include <opencv2/core/hal/interface.h>
#include <cmath>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgproc.hpp>
#include "common/eigen_types.hh"

namespace slam_learn {
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                     float resolution, const SE2& pose_submap) {
    if (image.data == nullptr) {
        // 创建一个纯白的图片
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }
        // 计算点的角度
        double real_angle = scan->angle_min + scan->angle_increment * i;
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);

        // 去掉一定角度的点
        if (real_angle < scan->angle_min + 30.0 * M_PI / 180 || real_angle > scan->angle_max - 30 * M_PI / 180) {
            continue;
        }
        // 将激光的xy转到换子图的坐标中
        Vec2d pose_in_submap = pose_submap.inverse() * (pose * Vec2d(x, y));
        // 栅格化, 并且转化为图像的中心
        int image_x = int(pose_in_submap[0] * resolution + image_size / 2);
        int image_y = int(pose_in_submap[1] * resolution + image_size / 2);
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            // 给每个像素绘制颜色
            image.at<cv::Vec3b>(image_x, image_y) = cv::Vec3b(color[0], color[1], color[2]);
        }
    }
    // 绘制机器人pose
    Vec2d pose_in_image =
        pose_submap.inverse() * (pose.translation() * (double)resolution + Vec2d(image_size / 2, image_size / 2));
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5, cv::Scalar(color[0], color[1], color[2]), 2);
}
}  // namespace slam_learn