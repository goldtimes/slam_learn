#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

// camera pose
struct Pose {
    Pose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) : Rwc(R), qwc(R), twc(t) {
    }

    // camera->world
    Eigen::Matrix3d Rwc;
    Eigen::Vector3d twc;
    Eigen::Quaterniond qwc;
};

int main(int argc, char** argv) {
    // 特征点的个数
    int featureNums = 20;
    // 相机姿态的个数
    int poseNums = 10;
    int dim = poseNums * 6 + featureNums * 3;
    double fx = 1;
    double fy = 1;
    Eigen::MatrixXd H(dim, dim);
    H.setZero();
    Eigen::VectorXd b(dim, 1);

    std::vector<Pose> camera_pose;
    double radius = 0;
    for (int n = 0; n < poseNums; ++n) {
        // 相机做圆周运动 1/4的圆弧
        double theta = n * 2 * M_PI / (poseNums * 4);
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R, t));
    }
    // 生成三维的点
    std::default_random_engine generator;
    std::vector<Eigen::Vector3d> points;
    for (int j = 0; j < featureNums; ++j) {
        std::uniform_real_distribution<double> xy_rand(-4, 4);
        std::uniform_real_distribution<double> z_rand(8.0, 10.0);
        double tx = xy_rand(generator);
        double ty = xy_rand(generator);
        double tz = z_rand(generator);
        Eigen::Vector3d Pworld(tx, ty, tz);
        points.push_back(Pworld);
        // 特征点在每个相机下的成像位置
        for (int i = 0; i < poseNums; ++i) {
            Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
            // 将世界点的转换到camera坐标系下
            Eigen::Vector3d Pc = Rcw * (Pworld - camera_pose[i].twc);

            double x = Pc(0);
            double y = Pc(1);
            double z = Pc(2);
            double z_2 = z * z;
            // 像素点对Pc的jacobian
            Eigen::Matrix<double, 2, 3> jacobian_uv_Pc;
            // clang-format off
            jacobian_uv_Pc << fx/z ,0, -x * fx / z_2,
                              0, fy / 2, -y * fy / z_2;
            // clang-format on
            // 对世界坐标下的jacobian
            // 对pose的jacobian
            Eigen::Matrix<double, 2, 3> jacobian_Pj = jacobian_uv_Pc * Rcw;
            Eigen::Matrix<double, 2, 6> jacobian_Ti;
            jacobian_Ti << -x * y * fx / z_2, (1 + x * x / z_2) * fx, -y / z * fx, fx / z, 0, -x * fx / z_2,
                -(1 + y * y / z_2) * fy, x * y / z_2 * fy, x / z * fy, 0, fy / z, -y * fy / z_2;

            H.block(i * 6, i * 6, 6, 6) += jacobian_Ti.transpose() * jacobian_Ti;
            /// 请补充完整作业信息矩阵块的计算
            // H.block(j*3 + 6*poseNums,j*3 + 6*poseNums,3,3) +=?????
            // H.block(i*6,j*3 + 6*poseNums, 6,3) += ???;
            H.block(j * 3 + 6 * poseNums, j * 3 + 6 * poseNums, 3, 3) += jacobian_Pj.transpose() * jacobian_Pj;
            H.block(i * 6, j * 3 + 6 * poseNums, 6, 3) += jacobian_Ti.transpose() * jacobian_Pj;
            H.block(j * 3 + 6 * poseNums, i * 6, 3, 6) += jacobian_Pj.transpose() * jacobian_Ti;
        }
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << svd.singularValues() << std::endl;

    return 0;
}