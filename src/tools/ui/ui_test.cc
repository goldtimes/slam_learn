#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include "tools/ui/pangolin_window.hh"

DEFINE_string(source, "./data/ch5/first.pcd", "第1个点云路径");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    slam_learn::ui::PangolinWindow ui;
    ui.Init();
    slam_learn::CloudPtr source(new slam_learn::PointCloudXYZI);
    pcl::io::loadPCDFile(fLS::FLAGS_source, *source);

    LOG(INFO) << "set state";
    ui.UpdateScan(source, slam_learn::SE3());

    LOG(INFO) << "waiting";
    sleep(60);
    ui.Quit();

    return 0;
}