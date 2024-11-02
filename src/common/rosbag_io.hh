#include <glog/logging.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <functional>
#include <map>
#include <utility>
#include "common/dataset_type.hh"
#include "common/sensors/gnss.hh"
#include "common/sensors/imu.hh"
#include "common/sensors/point_type.hh"
#include "lidar_slam/ch3//utm_convert.hh"
#include "livox_ros_driver/CustomMsg.h"
#include "rosbag/message_instance.h"
namespace slam_learn::rosbag_io {
/**
 * @brief rosbagio 会定义一个回调函数，然后被测试程序告诉rosbag io如何出来改测试的数据
 */
class RosbagIO {
   public:
    using ImuHandle = std::function<bool(const IMUPtr)>;
    using GNSSHandle = std::function<bool(const GNSSPtr)>;
    using PointCloud2Handle = std::function<bool(const sensor_msgs::PointCloud2::Ptr)>;
    using FullPointCloudHandle = std::function<bool(const FullCloudPtr)>;
    using LivoxHandle = std::function<bool(const livox_ros_driver::CustomMsg::ConstPtr)>;

    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
        assert(dataset_type_ != DatasetType::UNKNOWN);
    }
    // 读取rosbag文件中的数据
    void Go();

    RosbagIO& AddHandle(const std::string& topic_name, std::function<bool(const rosbag::MessageInstance&)> func) {
        process_func_[topic_name] = func;
        return *this;
    }
    // 定义一个处理2d激光雷达的函数
    RosbagIO& AddScan2DHandle(const std::string& topic_name,
                              std::function<bool(const sensor_msgs::LaserScanPtr)> func) {
        // 将topic和func放到一个map中，但是需要保证键的类型一致，值的类型一致，所以都可以抽象为rosbag::MessageIntance
        // 于是我们这里再抽象一层
        return AddHandle(topic_name, [&func](const rosbag::MessageInstance& m) -> bool {
            auto msg = m.instantiate<sensor_msgs::LaserScan>();
            if (msg == nullptr) {
                return false;
            } else {
                // 传递进来的函数，处理雷达消息
                // 然后process_func_[topic]放到Go当中处理
                return func(msg);
            }
        });
    }

    /// 根据数据集自动处理RTK消息
    RosbagIO& AddAutoRTKHandle(GNSSHandle f) {
        if (dataset_type_ == DatasetType::NCLT) {
            return AddHandle(nclt_rtk_topic, [f, this](const rosbag::MessageInstance& m) -> bool {
                auto msg = m.instantiate<sensor_msgs::NavSatFix>();
                if (msg == nullptr) {
                    return false;
                }

                GNSSPtr gnss(new GNSS(msg));
                ConvertGps2UTMOnlyTrans(*gnss);
                if (std::isnan(gnss->lat_lon_alt_[2])) {
                    // 貌似有Nan
                    return false;
                }

                return f(gnss);
            });
        } else {
            // TODO 其他数据集的RTK转换关系
        }
    }

    /// point cloud 2 的处理
    RosbagIO& AddPointCloud2Handle(const std::string& topic_name, PointCloud2Handle f) {
        return AddHandle(topic_name, [f](const rosbag::MessageInstance& m) -> bool {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg == nullptr) {
                return false;
            }
            return f(msg);
        });
    }

    /// livox msg 处理
    RosbagIO& AddLivoxHandle(LivoxHandle f) {
        return AddHandle(GetLidarTopicName(), [f, this](const rosbag::MessageInstance& m) -> bool {
            auto msg = m.instantiate<livox_ros_driver::CustomMsg>();
            if (msg == nullptr) {
                LOG(INFO) << "cannot inst: " << m.getTopic();
                return false;
            }
            return f(msg);
        });
    }

    void CleanProcessFunc() {
        process_func_.clear();
    }

    RosbagIO& AddImuHandle(ImuHandle f);

   private:
    /// 根据设定的数据集名称获取雷达名
    std::string GetLidarTopicName() const;

    /// 根据数据集名称确定IMU topic名称
    std::string GetIMUTopicName() const;

   private:
    // 文件路径
    std::string bag_file_;
    // 数据集
    DatasetType dataset_type_;
    std::map<std::string, std::function<bool(const rosbag::MessageInstance&)>> process_func_;
};
}  // namespace slam_learn::rosbag_io