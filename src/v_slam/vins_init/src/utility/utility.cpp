#include "v_slam/vins_init/include/utility/utility.h"
#include <iostream>
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g) {
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    // 这里利用重力向量和{0,0,1}向量构建了一个旋转矩阵
    // 这里就是先将向量旋转到z轴上向的东北天坐标，然后此时yaw角不是对齐，获取yaw角之后，补偿回去就可以得到对齐的坐标
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
