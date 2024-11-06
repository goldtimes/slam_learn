#include "lidar_slam/ch9/keyframe.hh"
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iomanip>
#include <istream>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/sensors/point_type.hh"

namespace slam_learn::mapping {
/// 将本帧点云存盘，从内存中清除
void Keyframe::SaveAndUnloadScan(const std::string& path) {
    if (cloud_) {
        lidar_utils::SaveCloudToFile(path + "/" + std::to_string(id_) + ".pcd", *cloud_);
        cloud_ = nullptr;
    }
}

void Keyframe::LoadScan(const std::string& path) {
    cloud_.reset(new PointCloudXYZI);
    pcl::io::loadPCDFile(path + "/" + std::to_string(id_) + ".pcd", *cloud_);
}

/// 保存至文本文件
void Keyframe::Save(std::ostream& os) {
    auto save_se3 = [](std::ostream& f, const SE3& pose) {
        auto q = pose.so3().unit_quaternion();
        Vec3d t = pose.translation();
        f << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
    };
    os << id_ << " " << std::setprecision(18) << timestamped_ << " " << rtk_heading_valid_ << " " << rtk_valid_ << " "
       << rtk_inlier_ << " ";
    // 保存位姿
    save_se3(os, lidar_pose_);
    save_se3(os, rtk_pose_);
    save_se3(os, opti_pose_1_);
    save_se3(os, opti_pose_2_);
    os << std::endl;
}

/// 从文件读取
void Keyframe::Load(std::istream& is) {
    is >> id_ >> timestamped_ >> rtk_heading_valid_ >> rtk_valid_ >> rtk_inlier_;
    auto load_se3 = [](std::istream& f) {
        SE3 ret;
        double q[4];
        double t[3];
        f >> t[0] >> t[1] >> t[2] >> q[0] >> q[1] >> q[2] >> q[3];
        return SE3(Quatd(q[0], q[1], q[2], q[3]), Vec3d(t[0], t[1], t[2]));
    };
    lidar_pose_ = load_se3(is);
    rtk_pose_ = load_se3(is);
    opti_pose_1_ = load_se3(is);
    opti_pose_2_ = load_se3(is);
}

bool LoadKeyFrames(const std::string& path, std::map<int, std::shared_ptr<Keyframe>>& keyframes) {
    std::ifstream fin(path);
    if (!fin) {
        return false;
    }
    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            break;
        }
        std::stringstream ss;
        ss << line;
        auto kf = std::make_shared<Keyframe>();
        kf->Load(ss);
        keyframes.emplace(kf->id_, kf);
    }
    LOG(INFO) << "Loaded kfs: " << keyframes.size();
    return true;
}
}  // namespace slam_learn::mapping
