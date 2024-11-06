#pragma once
#include <map>
#include <memory>
#include "common/eigen_types.hh"
#include "common/sensors//point_type.hh"

namespace slam_learn::mapping {
struct Keyframe {
    Keyframe() {
    }
    Keyframe(double time, unsigned long id, const SE3& lidar_pose, CloudPtr cloud)
        : timestamped_(time), id_(id), lidar_pose_(lidar_pose), cloud_(cloud) {
    }

    /// 将本帧点云存盘，从内存中清除
    void SaveAndUnloadScan(const std::string& path);

    void LoadScan(const std::string& path);

    /// 保存至文本文件
    void Save(std::ostream& os);

    /// 从文件读取
    void Load(std::istream& is);

    double timestamped_ = 0.0;
    // using IdType = unsigned long;
    unsigned long id_;
    SE3 lidar_pose_;
    SE3 rtk_pose_;
    SE3 opti_pose_1_;
    SE3 opti_pose_2_;
    // 双天线的方案
    bool rtk_heading_valid_ = false;
    // 原始值是否有效
    bool rtk_valid_ = true;
    // 优化过程rtk的值是否正常
    bool rtk_inlier_ = true;

    CloudPtr cloud_ = nullptr;
};

bool LoadKeyFrames(const std::string& path, std::map<unsigned long, std::shared_ptr<Keyframe>>& keyframes);

}  // namespace slam_learn::mapping

using KeyFramePtr = std::shared_ptr<slam_learn::mapping::Keyframe>;