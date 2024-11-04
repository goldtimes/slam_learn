#include "lidar_slam/ch7/direct_ndt_lo.hh"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include "common/math_utils.hh"
#include "common/sensors/point_type.hh"

namespace slam_learn::ndt_lo {
/**
 * 往LO中增加一个点云
 * @param scan  当前帧点云
 * @param pose 估计pose
 */
void DirectNDTLO::AddCloud(CloudPtr scan, SE3& pose) {
    if (submap_ == nullptr) {
        // 点云
        submap_.reset(new PointCloudXYZI);
        *submap_ += *scan;
        // 状态
        pose = SE3();
        last_kf_pose = pose;
        // ndt 设置目标点云
        if (options_.use_pcl_ndt) {
            ndt_pcl_.setInputTarget(submap_);
        } else {
            ndt_.SetTarget(submap_);
        }
        return;
    }
    // 配准
    pose = AlignWithLocalMap(scan);
    CloudPtr scan_world(new PointCloudXYZI);
    pcl::transformPointCloud(*scan, *scan_world, pose.matrix().cast<float>());
    // 判断是否为keyframe
    if (IsKeyframe(pose)) {
        last_kf_pose = pose;
        // 更新submap
        keyframes_buffer.emplace_back(scan_world);
        if (keyframes_buffer.size() > options_.num_kfs) {
            keyframes_buffer.pop_front();
        }
        submap_.reset(new PointCloudXYZI);
        for (const auto& cloud : keyframes_buffer) {
            *submap_ += *cloud;
        }
        if (options_.use_pcl_ndt) {
            ndt_pcl_.setInputTarget(submap_);
        } else {
            ndt_.SetTarget(submap_);
        }
    }
    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(pose, scan_world);
    }
}

/// 存储地图(viewer里）
void DirectNDTLO::SaveMap(const std::string& map_path) {
    if (viewer_) {
        viewer_->SaveMap(map_path);
    }
}

/// 与local map进行配准
SE3 DirectNDTLO::AlignWithLocalMap(CloudPtr scan) {
    if (options_.use_pcl_ndt) {
        ndt_pcl_.setInputCloud(scan);
    } else {
        ndt_.SetSource(scan);
    }
    CloudPtr output(new PointCloudXYZI);

    SE3 guess;
    bool align_success = true;
    if (estimated_poses_.size() < 2) {
        if (options_.use_pcl_ndt) {
            ndt_pcl_.align(*output, guess.matrix().cast<float>());
            guess = math::Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
        } else {
            align_success = ndt_.AlignNdt(guess);
        }
    } else {
        // 从最近两个pose来推断 认为是匀速运动
        SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
        SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
        guess = T1 * (T2.inverse() * T1);
        if (options_.use_pcl_ndt) {
            ndt_pcl_.align(*output, guess.matrix().cast<float>());
            guess = math::Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
        } else {
            align_success = ndt_.AlignNdt(guess);
        }
    }
    LOG(INFO) << "pose: " << guess.translation().transpose() << ", "
              << guess.so3().unit_quaternion().coeffs().transpose();

    if (options_.use_pcl_ndt) {
        LOG(INFO) << "trans prob: " << ndt_pcl_.getTransformationProbability();
    }

    estimated_poses_.emplace_back(guess);
    return guess;
}

/// 判定是否为关键帧
bool DirectNDTLO::IsKeyframe(const SE3& current_pose) {
    SE3 delta_pose = last_kf_pose.inverse() * current_pose;
    if (delta_pose.translation().norm() > options_.kf_distance ||
        delta_pose.so3().log().norm() > options_.kf_angle * math::kDEG2RAD) {
        return true;
    }
    return false;
}
}  // namespace slam_learn::ndt_lo