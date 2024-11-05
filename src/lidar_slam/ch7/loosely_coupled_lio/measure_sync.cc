#include "lidar_slam/ch7/loosely_coupled_lio/measure_sync.hh"

namespace slam_learn::loosely {
/// 初始化
void MessageSync::Init(const std::string& yaml) {
    cloud_convert_->LoadFromYAML(yaml);
}

bool MessageSync::Sync() {
    // 检查传感器队列是否为空
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }
    // 取出雷达
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_begin_time = time_buffer_.front();
        lidar_end_time_ = measures_.lidar_begin_time + measures_.lidar_->points.back().time / double(1e3);
        measures_.lidar_end_time = lidar_end_time_;
        lidar_pushed_ = true;
    }
    // 当imu时间戳小于雷达结束时间时
    if (last_timestamped_imu_ < lidar_end_time_) {
        return false;
    }

    // 处理消息
    double imu_time = imu_buffer_.front()->timestamp_;
    measures_.imu_.clear();
    // imu不为空的时候，并且imu时间戳小于雷达结束时
    while (!imu_buffer_.empty() && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp_;
        // 同步结束
        if (imu_time > lidar_end_time_) {
            break;
        }
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    if (callback_) {
        callback_(measures_);
    }
    return true;
}
}  // namespace slam_learn::loosely