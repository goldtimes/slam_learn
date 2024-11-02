#include "common/rosbag_io.hh"
#include "rosbag/bag.h"
#include "rosbag/message_instance.h"
#include "rosbag/view.h"

namespace slam_learn::rosbag_io {

std::string RosbagIO::GetLidarTopicName() const {
    if (dataset_type_ == DatasetType::NCLT) {
        return nclt_lidar_topic;
    }
    if (dataset_type_ == DatasetType::ULHK) {
        return ulhk_lidar_topic;
    }
    if (dataset_type_ == DatasetType::WXB_3D) {
        return wxb_lidar_topic;
    }
    if (dataset_type_ == DatasetType::UTBM) {
        return utbm_lidar_topic;
    }
    if (dataset_type_ == DatasetType::AVIA) {
        return avia_lidar_topic;
    }
}

std::string RosbagIO::GetIMUTopicName() const {
    if (dataset_type_ == DatasetType::ULHK) {
        return ulhk_imu_topic;
    } else if (dataset_type_ == DatasetType::UTBM) {
        return utbm_imu_topic;
    } else if (dataset_type_ == DatasetType::NCLT) {
        return nclt_imu_topic;
    } else if (dataset_type_ == DatasetType::WXB_3D) {
        return wxb_imu_topic;
    } else if (dataset_type_ == DatasetType::AVIA) {
        return avia_imu_topic;
    } else {
        LOG(ERROR) << "cannot load imu topic name of dataset " << int(dataset_type_);
    }

    return "";
}

RosbagIO &RosbagIO::AddImuHandle(RosbagIO::ImuHandle f) {
    return AddHandle(GetIMUTopicName(), [&f, this](const rosbag::MessageInstance &m) -> bool {
        auto msg = m.template instantiate<sensor_msgs::Imu>();
        if (msg == nullptr) {
            return false;
        }

        IMUPtr imu;
        if (dataset_type_ == DatasetType::AVIA) {
            // Livox内置imu的加计需要乘上重力常数
            imu =
                std::make_shared<IMU>(msg->header.stamp.toSec(),
                                      Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                      Vec3d(msg->linear_acceleration.x * 9.80665, msg->linear_acceleration.y * 9.80665,
                                            msg->linear_acceleration.z * 9.80665));
        } else {
            imu = std::make_shared<IMU>(
                msg->header.stamp.toSec(),
                Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
        }
        return f(imu);
    });
}

void RosbagIO::Go() {
    rosbag::Bag bag(bag_file_);
    LOG(INFO) << "running in " << bag_file_ << ", reg process func: " << process_func_.size();

    if (!bag.isOpen()) {
        LOG(ERROR) << "cannot open " << bag_file_;
        return;
    }
    // 读取每个消息
    auto view = rosbag::View(bag);
    for (const rosbag::MessageInstance &m : view) {
        auto iter = process_func_.find(m.getTopic());
        // 存在该topic的回调函数
        if (iter != process_func_.end()) {
            // 取出函数并调用，真正处理数据
            iter->second(m);
        }
    }
    bag.close();
    LOG(INFO) << "bag " << bag_file_ << " finished.";
}
}  // namespace slam_learn::rosbag_io