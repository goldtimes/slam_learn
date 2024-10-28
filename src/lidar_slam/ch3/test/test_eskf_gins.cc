#include "common/io_utils.hh"
#include "lidar_slam/ch3/eskf.hpp"
#include "lidar_slam/ch3/static_imu_init.hh"
#include "lidar_slam/ch3/utm_convert.hh"
#include "tools/ui/pangolin_window.hh"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>
#include <memory>

DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角(角度)");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");
DEFINE_bool(with_odom, false, "是否加入轮速计信息");

int main(int argc, char** argv) {
    using namespace slam_learn;
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }
    slam_learn::eskf::ESKFD eskf;
    slam_learn::imu_init::StaticIMUInit imu_init;
    slam_learn::io_utils::TxtIO io(FLAGS_txt_path);
    // 安装的外惨
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);
    // 定义三个保存数据的函数
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
    std::ofstream fout("./data/ch3/gins.txt");
    bool imu_inited = false, gnss_inited = false;
    std::shared_ptr<slam_learn::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<slam_learn::ui::PangolinWindow>();
        ui->Init();
    }

    /// 设置各类回调函数
    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();
    io.SetIMUProcessFunc([&](const IMU& imu) {
          /// imu初始化
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          /// 需要IMU初始化
          if (!imu_inited) {
              // 读取初始零偏，设置ESKF
              eskf::ESKFD::Options options;
              // 噪声由初始化器估计
              options.gyro_var_ = std::sqrt(imu_init.GetCovGyro()[0]);
              options.acce_var_ = std::sqrt(imu_init.GetCovAcce()[0]);
              eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
              imu_inited = true;
              return;
          }

          if (!gnss_inited) {
              /// 等待有效的RTK数据
              return;
          }

          /// GNSS 也接收到之后，再开始进行预测
          eskf.Predict(imu);

          /// predict就会更新ESKF，所以此时就可以发送数据
          auto state = eskf.GetNominalState();
          if (ui) {
              // 更新一次ui状态
              ui->UpdateNavState(state);
          }

          /// 记录数据以供绘图
          save_result(fout, state);

          usleep(1e3);
      })
        .SetGNSSProcessFunc([&](const GNSS& gnss) {
            /// GNSS 处理函数
            if (!imu_inited) {
                // 需要imu初始化成功后再处理gnss信息
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

            // 要求RTK heading有效，才能合入ESKF
            eskf.ObserveGps(gnss_convert);

            auto state = eskf.GetNominalState();
            if (ui) {
                ui->UpdateNavState(state);
            }
            save_result(fout, state);

            gnss_inited = true;
        })
        .SetOdomProcessFunc([&](const Odom& odom) {
            /// Odom 处理函数，本章Odom只给初始化使用
            imu_init.AddOdom(odom);
            if (FLAGS_with_odom && imu_inited && gnss_inited) {
                eskf.ObserveWheelSpeed(odom);
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