#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "common/g2o_types.hh"
#include "common/io_utils.hh"
#include "common/math_utils.hh"
#include "lidar_slam/ch3/eskf.hpp"
#include "lidar_slam/ch3/static_imu_init.hh"
#include "lidar_slam/ch3/utm_convert.hh"
#include "lidar_slam/ch4/imu_preintegration.hh"

// g2o的求解器
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include "lidar_slam/ch4/gins_pre_integ.hh"
#include "tools/ui/pangolin_window.hh"

/**
 * 运行由预积分构成的GINS系统
 */
DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, false, "是否显示图形界面");
DEFINE_bool(debug, true, "是否打印调试信息");

int main(int argc, char** argv) {
    using namespace slam_learn;
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    // 初始化器
    imu_init::StaticIMUInit imu_init;  // 使用默认配置

    io_utils::TxtIO io(fLS::FLAGS_txt_path);
    Vec2d antenna_pos(fLD::FLAGS_antenna_pox_x, fLD::FLAGS_antenna_pox_y);

    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamped_ << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        fout << std::endl;
    };

    std::ofstream fout("./data/ch4/gins_preintg.txt");
    bool imu_inited = false, gnss_inited = false;

    GinsPreInteg::Options gins_options;
    gins_options.verbose_ = FLAGS_debug;
    GinsPreInteg gins(gins_options);

    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    std::shared_ptr<ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<ui::PangolinWindow>();
        ui->Init();
    }

    /// 设置各类回调函数
    io.SetIMUProcessFunc([&](const IMU& imu) {
          /// IMU 处理函数
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          /// 需要IMU初始化
          if (!imu_inited) {
              // 读取初始零偏，设置GINS
              GinsPreInteg::Options options;
              options.preinteg_options_.init_bg_ = imu_init.GetInitBg();
              options.preinteg_options_.init_ba_ = imu_init.GetInitBa();
              options.gravity_ = imu_init.GetGravity();
              gins.SetOptions(options);
              imu_inited = true;
              return;
          }

          if (!gnss_inited) {
              /// 等待有效的RTK数据
              return;
          }

          /// GNSS 也接收到之后，再开始进行预测
          gins.AddImu(imu);

          auto state = gins.GetState();
          save_result(fout, state);
          if (ui) {
              ui->UpdateNavState(state);
              usleep(5e2);
          }
      })
        .SetGNSSProcessFunc([&](const GNSS& gnss) {
            /// GNSS 处理函数
            if (!imu_inited) {
                return;
            }

            GNSS gnss_convert = gnss;
            if (!ConvertGps2UTM(gnss_convert, antenna_pos, FLAGS_antenna_angle) || !gnss_convert.heading_valid_) {
                return;
            }

            /// 去掉原点
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
            }
            gnss_convert.utm_pose_.translation() -= origin;

            gins.AddGnss(gnss_convert);

            auto state = gins.GetState();
            save_result(fout, state);
            if (ui) {
                ui->UpdateNavState(state);
                usleep(1e3);
            }
            gnss_inited = true;
        })
        .SetOdomProcessFunc([&](const Odom& odom) {
            imu_init.AddOdom(odom);

            if (imu_inited && gnss_inited) {
                gins.AddOdom(odom);
            }
        })
        .Go();

    while (ui && !ui->ShouldQuit()) {
        usleep(1e5);
    }
    if (ui) {
        ui->Quit();
    }
    return 0;
}