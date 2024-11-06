#include "lidar_slam/ch9/loopclosure.hh"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cstdlib>
#include <execution>
#include <vector>

#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/sensors/point_type.hh"
#include "lidar_slam/ch9/keyframe.hh"

namespace slam_learn::mapping {
LoopClosure::LoopClosure(const std::string& config_yaml) : yaml_(config_yaml) {
}

bool LoopClosure::Init() {
    if (!LoadKeyFrames("/home/kilox/hang_ws/src/slam_learn/data/ch9/keyframes.txt", keyframes_)) {
        LOG(ERROR) << "cannot load keyframes";
        return false;
    }
    LOG(INFO) << "keyframes: " << keyframes_.size();

    auto yaml = YAML::LoadFile(yaml_);
    min_id_interval_ = yaml["loop_closing"]["min_id_interval"].as<int>();
    min_distance_ = yaml["loop_closing"]["min_distance"].as<double>();
    skip_id_ = yaml["loop_closing"]["skip_id"].as<int>();
    ndt_score_th_ = yaml["loop_closing"]["ndt_score_th"].as<double>();
    return true;
}
void LoopClosure::Run() {
    DetectLoopCandidates();
    ComputeLoopCandidates();

    SaveResults();
}

/// 提取回环的候选
void LoopClosure::DetectLoopCandidates() {
    KeyFramePtr check_first = nullptr;
    KeyFramePtr check_second = nullptr;

    LOG(INFO) << "detecting loop candidates from pose in stage 1";

    for (auto iter_first = keyframes_.begin(); iter_first != keyframes_.end(); ++iter_first) {
        // 获取keyframe
        auto kf_first = iter_first->second;
        if (check_first != nullptr && abs((int)kf_first->id_ - (int)check_first->id_) <= skip_id_) {
            // 如果之前已经有回环的候选帧了，我们跳过5帧之内的关键帧
            continue;
        }

        for (auto iter_second = iter_first; iter_second != keyframes_.end(); ++iter_second) {
            auto kf_second = iter_second->second;
            if (check_second != nullptr && abs((int)kf_second->id_ - (int)check_second->id_) <= skip_id_) {
                // 如果之前已经有回环的候选帧了，我们跳过5帧之内的关键帧
                continue;
            }
            // 关键帧的id太近，不考虑回环
            if (abs((int)kf_first->id_ - (int)kf_second->id_) < min_id_interval_) {
                continue;
            }
            // 这里我们得到两个id较大的帧
            Vec3d dt = kf_first->opti_pose_1_.translation() - kf_first->opti_pose_1_.translation();
            double dxy = dt.head<2>().norm();
            double range_th = min_distance_;
            // 但是距离较小，有可能是回环
            if (dxy < range_th) {
                LoopCandidate tmp_candidate(kf_first->id_, kf_second->id_,
                                            kf_first->opti_pose_1_.inverse() * kf_first->opti_pose_1_);
                loop_candiates_.emplace_back(tmp_candidate);
                check_first = kf_first;
                check_second = kf_second;
            }
        }
    }
    LOG(INFO) << "detected candidates: " << loop_candiates_.size();
}

/// 计算回环候选帧的相对运动
void LoopClosure::ComputeLoopCandidates() {
    // 执行并行计算，并且做ndt匹配
    std::for_each(std::execution::par_unseq, loop_candiates_.begin(), loop_candiates_.end(),
                  [this](LoopCandidate& candidate) { ComputeForCandidate(candidate); });
    // 保留成功的候选
    std::vector<LoopCandidate> succ_candidates;
    for (const auto& lc : loop_candiates_) {
        if (lc.ndt_score_ > ndt_score_th_) {
            succ_candidates.emplace_back(lc);
        }
    }
    LOG(INFO) << "success: " << succ_candidates.size() << "/" << loop_candiates_.size();

    loop_candiates_.swap(succ_candidates);
}

/// 计算一个回环候选是否成立
void LoopClosure::ComputeForCandidate(LoopCandidate& c) {
    LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    const int submap_idx_range = 40;
    KeyFramePtr kf1 = keyframes_.at(c.idx1_);
    KeyFramePtr kf2 = keyframes_.at(c.idx2_);
    // 构建关键帧周围的子地图
    auto build_submap = [&](int given_id, bool build_in_world) {
        CloudPtr submap(new PointCloudXYZI);
        // idx (-40,40,4)
        for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
            int id = idx + given_id;
            // 判断id是否合理
            if (id < 0) {
                continue;
            }
            auto iter = keyframes_.find(id);
            if (iter == keyframes_.end()) {
                continue;
            }
            auto kf = iter->second;
            // load pcd
            CloudPtr cloud(new PointCloudXYZI);
            pcl::io::loadPCDFile("./data/ch9/" + std::to_string(id) + ".pcd", *cloud);
            if (cloud->empty()) {
                continue;
            }
            // 转到世界坐标系
            SE3 Twb = kf->opti_pose_1_;
            if (!build_in_world) {
                Twb = keyframes_.at(given_id)->opti_pose_1_.inverse() * Twb;
            }

            CloudPtr cloud_trans(new PointCloudXYZI);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());

            *submap += *cloud_trans;
        }
        return submap;
    };

    auto submap_kf1 = build_submap(kf1->id_, true);
    CloudPtr submap_kf2(new PointCloudXYZI);
    // scan to map
    pcl::io::loadPCDFile("./data/ch9/" + std::to_string(kf2->id_) + ".pcd", *submap_kf2);
    if (submap_kf1->empty() || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        return;
    }
    // ndt
    pcl::NormalDistributionsTransform<PointXYZI, PointXYZI> ndt;

    ndt.setTransformationEpsilon(0.05);
    ndt.setStepSize(0.7);
    ndt.setMaximumIterations(40);
    // 初始位姿
    Mat4f Tw2 = kf2->opti_pose_1_.matrix().cast<float>();

    /// 不同分辨率下的匹配
    CloudPtr output(new PointCloudXYZI);
    std::vector<double> res{10.0, 5.0, 4.0, 3.0};
    for (auto& r : res) {
        ndt.setResolution(r);
        // 降采样
        auto rough_map1 = lidar_utils::VoxelCloud(submap_kf1, r * 0.1);
        auto rough_map2 = lidar_utils::VoxelCloud(submap_kf2, r * 0.1);
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        ndt.align(*output, Tw2);
        Tw2 = ndt.getFinalTransformation();
    }

    Mat4d T = Tw2.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);
    // 更新Tij_;
    c.Tij_ = kf1->opti_pose_1_.inverse() * SE3(q, t);
    // 更新分数
    c.ndt_score_ = ndt.getTransformationProbability();
}

/// 保存计算结果
void LoopClosure::SaveResults() {
    auto save_SE3 = [](std::ostream& f, SE3 pose) {
        auto q = pose.so3().unit_quaternion();
        Vec3d t = pose.translation();
        f << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
    };

    std::ofstream fout("./data/ch9/loops.txt");
    for (const auto& lc : loop_candiates_) {
        fout << lc.idx1_ << " " << lc.idx2_ << " " << lc.ndt_score_ << " ";
        save_SE3(fout, lc.Tij_);
        fout << std::endl;
    }
}

}  // namespace slam_learn::mapping
