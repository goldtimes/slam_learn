#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "common/io_utils.hh"
#include "common/math_utils.hh"
#include "lidar_slam/ch3/eskf.hpp"
#include "lidar_slam/ch4/imu_preintegration.hh"

DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角(角度)");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, false, "是否显示图形界面");
using namespace slam_learn;

/**
 * @brief 测试恒定的角速度情况下，预积分的值和eskf的值
 */
TEST(PREINTEGRATION_TEST, ROTATION_TEST) {
    double imu_time_span = 0.01;
    Vec3d constant_omega(0, 0, M_PI);
    Vec3d gravity(0, 0, -9.81);  // Z轴向上
    NavStated start_status(0);
    NavStated end_status(1);
    IMUPreintegration pre_integ;

    // 直接积分
    SO3 R;
    Vec3d v = Vec3d::Zero();
    Vec3d t = Vec3d::Zero();

    for (int i = 0; i <= 100; ++i) {
        double imu_time = imu_time_span * i;
        Vec3d acce = -gravity;
        // 计算0.01之间的预积分值
        pre_integ.Integrate(IMU(imu_time, constant_omega, acce), imu_time_span);
        NavStated this_status = pre_integ.Predict(start_status, gravity);
        // 直接积分
        t = t + v * imu_time_span + 0.5 * gravity * imu_time_span * imu_time_span +
            0.5 * (R * acce) * imu_time_span * imu_time_span;
        v = v + gravity * imu_time_span + (R * acce) * imu_time_span;
        R = R * Sophus::SO3d::exp(constant_omega * imu_time_span);

        // 验证在简单情况下，直接积分和预积分结果相等
        EXPECT_NEAR(t[0], this_status.p_[0], 1e-2);
        EXPECT_NEAR(t[1], this_status.p_[1], 1e-2);
        EXPECT_NEAR(t[2], this_status.p_[2], 1e-2);

        EXPECT_NEAR(v[0], this_status.v_[0], 1e-2);
        EXPECT_NEAR(v[1], this_status.v_[1], 1e-2);
        EXPECT_NEAR(v[2], this_status.v_[2], 1e-2);

        EXPECT_NEAR(R.unit_quaternion().x(), this_status.R_.unit_quaternion().x(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().y(), this_status.R_.unit_quaternion().y(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().z(), this_status.R_.unit_quaternion().z(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().w(), this_status.R_.unit_quaternion().w(), 1e-4);
    }

    end_status = pre_integ.Predict(start_status);
    LOG(INFO) << "preinteg result: ";
    LOG(INFO) << "end rotation: \n" << end_status.R_.matrix();
    LOG(INFO) << "end trans: \n" << end_status.p_.transpose();
    LOG(INFO) << "end v: \n" << end_status.v_.transpose();

    LOG(INFO) << "direct integ result: ";
    LOG(INFO) << "end rotation: \n" << R.matrix();
    LOG(INFO) << "end trans: \n" << t.transpose();
    LOG(INFO) << "end v: \n" << v.transpose();
    SUCCEED();
}

TEST(PREINTEGRATION_TEST, ACCELERATION_TEST) {
    // 测试在恒定加速度运行下的预积分情况
    double imu_time_span = 0.01;     // IMU测量间隔
    Vec3d gravity(0, 0, -9.8);       // Z 向上，重力方向为负
    Vec3d constant_acce(0.1, 0, 0);  // x 方向上的恒定加速度

    NavStated start_status(0), end_status(1.0);
    IMUPreintegration pre_integ;

    // 对比直接积分
    Sophus::SO3d R;
    Vec3d t = Vec3d::Zero();
    Vec3d v = Vec3d::Zero();

    for (int i = 1; i <= 100; ++i) {
        double time = imu_time_span * i;
        Vec3d acce = constant_acce - gravity;
        pre_integ.Integrate(IMU(time, Vec3d::Zero(), acce), imu_time_span);
        NavStated this_status = pre_integ.Predict(start_status, gravity);

        t = t + v * imu_time_span + 0.5 * gravity * imu_time_span * imu_time_span +
            0.5 * (R * acce) * imu_time_span * imu_time_span;
        v = v + gravity * imu_time_span + (R * acce) * imu_time_span;

        // 验证在简单情况下，直接积分和预积分结果相等
        EXPECT_NEAR(t[0], this_status.p_[0], 1e-2);
        EXPECT_NEAR(t[1], this_status.p_[1], 1e-2);
        EXPECT_NEAR(t[2], this_status.p_[2], 1e-2);

        EXPECT_NEAR(v[0], this_status.v_[0], 1e-2);
        EXPECT_NEAR(v[1], this_status.v_[1], 1e-2);
        EXPECT_NEAR(v[2], this_status.v_[2], 1e-2);

        EXPECT_NEAR(R.unit_quaternion().x(), this_status.R_.unit_quaternion().x(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().y(), this_status.R_.unit_quaternion().y(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().z(), this_status.R_.unit_quaternion().z(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().w(), this_status.R_.unit_quaternion().w(), 1e-4);
    }

    end_status = pre_integ.Predict(start_status);
    LOG(INFO) << "preinteg result: ";
    LOG(INFO) << "end rotation: \n" << end_status.R_.matrix();
    LOG(INFO) << "end trans: \n" << end_status.p_.transpose();
    LOG(INFO) << "end v: \n" << end_status.v_.transpose();

    LOG(INFO) << "direct integ result: ";
    LOG(INFO) << "end rotation: \n" << R.matrix();
    LOG(INFO) << "end trans: \n" << t.transpose();
    LOG(INFO) << "end v: \n" << v.transpose();
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