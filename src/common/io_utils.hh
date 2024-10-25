#pragma once

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <fstream>
#include <functional>
#include <utility>
#include "common/sensors/gnss.hh"
#include "common/sensors/imu.hh"
#include "common/sensors/odom.hh"
// #include

namespace slam_learn::io_utils {
class TxtIO {
   public:
    using IMUProcessFuncType = std::function<void(const IMU &)>;
    using OdomProcessFuncType = std::function<void(const Odom &)>;
    using GNSSProcessFuncType = std::function<void(const GNSS &)>;
    TxtIO(const std::string &file_path) : fin(file_path) {
    }

    TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
        imu_proc_ = std::move(imu_proc);
        return *this;
    }

    TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc) {
        odom_proc_ = std::move(odom_proc);
        return *this;
    }

    TxtIO &SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
        gnss_proc_ = std::move(gnss_proc);
        return *this;
    }

    // 遍历文件内容，调用回调函数
    void Go();

   private:
    std::ifstream fin;

    IMUProcessFuncType imu_proc_;
    OdomProcessFuncType odom_proc_;
    GNSSProcessFuncType gnss_proc_;
};
}  // namespace slam_learn::io_utils
