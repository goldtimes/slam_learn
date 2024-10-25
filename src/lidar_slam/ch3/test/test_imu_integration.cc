#include <glog/logging.h>
#include <iomanip>
#include "common/io_utils.hh"
#include "lidar_slam/ch3/imu_integration.hh"
#include "tools/ui/pangolin_window.hh"

DEFINE_string(imu_txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_bool(with_ui, true, "是否显示图形界面");

int main(int argc, char** argv) {
    using namespace slam_learn;
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    slam_learn::io_utils::TxtIO io(FLAGS_imu_txt_path);

    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    IMUIntegration imu_integ(gravity, init_bg, init_ba);

    // ui
    std::shared_ptr<slam_learn::ui::PangolinWindow> ui_ptr = nullptr;
    if (FLAGS_with_ui) {
        ui_ptr = std::make_shared<slam_learn::ui::PangolinWindow>();
        ui_ptr->Init();
    }

    /// 记录结果
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,
                          const Vec3d& p) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };

    std::ofstream fout("./data/ch3/state.txt");
    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui_ptr](const IMU& imu) {
          imu_integ.AddIMU(imu);
          save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP());
          if (ui_ptr) {
              ui_ptr->UpdateNavState(imu_integ.GetNavState());
              usleep(1e2);
          }
      })
        .Go();

    // 打开了可视化的话，等待界面退出
    while (ui_ptr && !ui_ptr->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui_ptr) {
        ui_ptr->Quit();
    }

    return 0;
}