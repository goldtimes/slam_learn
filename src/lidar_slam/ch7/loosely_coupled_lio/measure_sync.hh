#pragma once
#include "lidar_slam/ch7/loosely_coupled_lio/cloud_convert.hh"

#include <glog/logging.h>
#include <deque>
#include <functional>
#include <memory>
#include "common/sensors/imu.hh"
#include "common/sensors/point_type.hh"

namespace slam_learn {
struct MeasureGroup {
    MeasureGroup() {
        lidar_.reset(new FullPointCloudType);
    }
    double lidar_begin_time = 0.0;
    double lidar_end_time = 0.0;
    FullCloudPtr lidar_ = nullptr;
    std::deque<IMUPtr> imu_;
};

class MessageSync {
   public:
    using Callback = std::function<void(const MeasureGroup& group)>;

   public:
    /// 初始化
    void Init(const std::string& yaml);

    MessageSync(Callback func) : callback_(func), cloud_convert_(new CloudConvert) {
    }

    void ProcessIMU(IMUPtr& imu) {
        double timestamped = imu->timestamp_;
        if (timestamped < last_timestamped_imu_) {
            LOG(WARNING) << "imu loop back, clear buffer";
            imu_buffer_.clear();
        }
        last_timestamped_imu_ = timestamped;
        imu_buffer_.push_back(imu);
    }

    void ProcessCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
            LOG(ERROR) << "lidar loop back, clear buffer";
            lidar_buffer_.clear();
        }

        FullCloudPtr cloud(new FullPointCloudType());
        cloud_convert_->Process(msg, cloud);
        lidar_buffer_.push_back(cloud);
        time_buffer_.push_back(msg->header.stamp.toSec());
        last_timestamp_lidar_ = msg->header.stamp.toSec();

        Sync();
    }

    /// 处理Livox点云
    void ProcessCloud(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
            LOG(ERROR) << "lidar loop back, clear buffer";
            lidar_buffer_.clear();
        }

        FullCloudPtr cloud(new FullPointCloudType());
        cloud_convert_->Process(msg, cloud);
        lidar_buffer_.push_back(cloud);
        time_buffer_.push_back(msg->header.stamp.toSec());
        last_timestamp_lidar_ = msg->header.stamp.toSec();

        Sync();
    }

   private:
    /// 尝试同步IMU与激光数据，成功时返回true
    // 这里将数据同步之后
    // 就需要处理数据，但是这里不知道怎么处理，于是我们只需要定义一个回调函数
    bool Sync();

   private:
    //
    Callback callback_;
    // 同步的数据
    MeasureGroup measures_;
    double last_timestamped_imu_ = 0.0;
    double last_timestamp_lidar_ = 0.0;
    double lidar_end_time_ = 0.0;
    bool lidar_pushed_ = false;
    std::shared_ptr<CloudConvert> cloud_convert_ = nullptr;

    // 数据
    std::deque<FullCloudPtr> lidar_buffer_;
    std::deque<IMUPtr> imu_buffer_;
    std::deque<double> time_buffer_;
};
}  // namespace slam_learn::loosely