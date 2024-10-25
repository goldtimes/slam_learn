#include "common/io_utils.hh"
#include <glog/logging.h>
#include <string>
namespace slam_learn::io_utils {
// 遍历文件内容，调用回调函数
void TxtIO::Go() {
    if (!fin) {
        LOG(ERROR) << "未能找到文件";
        return;
    }
    // 循环直到文件尾部
    while (!fin.eof()) {
        // 读取每一行
        std::string line;
        std::getline(fin, line);
        // 如果为空行，跳过
        if (line.empty()) {
            continue;
        }
        // 注释行
        if (line[0] == '#') {
            continue;
        }
        // 读取数据,一般用stringstream来分割每行的字符
        std::stringstream ss;
        ss << line;
        std::string data_type;
        ss >> data_type;
        // 每读取一行，相应的传感器处理程序就处理一次数据
        if (data_type == "IMU" && imu_proc_) {
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            // imu_proc_(IMU(time, Vec3d(gx, gy, gz) * math::kDEG2RAD, Vec3d(ax, ay, az)));
            imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
        } else if (data_type == "ODOM" && odom_proc_) {
            double time, wl, wr;
            ss >> time >> wl >> wr;
            odom_proc_(Odom(time, wl, wr));
        } else if (data_type == "GNSS" && gnss_proc_) {
            double time, lat, lon, alt, heading;
            bool heading_valid;
            ss >> time >> lat >> lon >> alt >> heading >> heading_valid;
            gnss_proc_(GNSS(time, 4, Vec3d(lat, lon, alt), heading, heading_valid));
        }
    }
    LOG(INFO) << "done.";
}
}  // namespace slam_learn::io_utils
