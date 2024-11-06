
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "lidar_slam/ch9/loopclosure.hh"

DEFINE_string(config_yaml, "/home/kilox/hang_ws/src/slam_learn/config/mapping.yaml", "配置文件");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    slam_learn::mapping::LoopClosure lc(FLAGS_config_yaml);
    lc.Init();
    lc.Run();

    return 0;
}