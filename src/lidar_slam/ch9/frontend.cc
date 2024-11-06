#include "lidar_slam/ch9/frontend.hh"
#include "common/dataset_type.hh"
#include "common/eigen_types.hh"
#include "common/math_utils.hh"
#include "common/sensors/gnss.hh"
#include "common/sensors/imu.hh"
#include "lidar_slam/ch8/lio-iekf//lio_iekf.hh"

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>
#include <vector>
#include "common/rosbag_io.hh"
#include "lidar_slam/ch9/keyframe.hh"
#include "sensor_msgs/PointCloud2.h"

namespace slam_learn::mapping {
Frontend::Frontend(const std::string& config_yaml) {
    config_yaml_ = config_yaml;
}

// 初始化，创建LIO对象，检查数据存在性
bool Frontend::Init() {
    LOG(INFO) << "load yaml from " << config_yaml_;
    auto yaml = YAML::LoadFile(config_yaml_);
    try {
        auto n = yaml["bag_path"];
        LOG(INFO) << Dump(n);
        bag_path_ = yaml["bag_path"].as<std::string>();
        lio_yaml_ = yaml["lio_yaml"].as<std::string>();
    } catch (...) {
        LOG(ERROR) << "failed to parse yaml";
        return false;
    }

    system("rm -rf ./data/ch9/*.pcd");
    system("rm -rf ./data/ch9/keyframes.txt");

    LioIEKF::Options options;
    options.with_ui_ = false;  // 跑建图不需要打开前端UI
    lio_ = std::make_shared<LioIEKF>(options);
    lio_->Init(lio_yaml_);
    return true;
}

/// 运行前端
void Frontend::Run() {
    slam_learn::rosbag_io::RosbagIO bag_io(bag_path_, DatasetType::NCLT);
    // 先读取所有的rtk数据
    // rosbag io 负责读取和生成gnss数据
    bag_io.AddAutoRTKHandle([this](GNSSPtr gnss) {
        // 这里只需要定义生成的数据如何处理
        // 放到队列中
        gnss_datas_.emplace(gnss->unix_time_, gnss);
        return true;
    });
    bag_io.CleanProcessFunc();
    RemoveMapOrigin();
    // 运行lio
    bag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            lio_->PCLCallBack(cloud);
            ExtractKeyFrame(lio_->GetCurrentState());
            return true;
        })
        .AddImuHandle([&](IMUPtr imu) -> bool {
            lio_->IMUCallBack(imu);
            return true;
        })
        .Go();
    lio_->Finish();

    // 保存运行结果
    SaveKeyframes();

    LOG(INFO) << "done.";
}

void Frontend::RemoveMapOrigin() {
    // 将第一帧有效的gnss作为原点
    if (gnss_datas_.empty()) {
        return;
    }
    bool origin_set = false;
    for (auto& gnss : gnss_datas_) {
        // 固定解的情况下
        if (gnss.second->status_ == GpsStatusType::GNSS_FIXED_SOLUTION) {
            map_origin_ = gnss.second->utm_pose_.translation();
            origin_set = true;
            LOG(INFO) << "map origin is set to " << map_origin_.transpose();
            // 写入yaml
            auto yaml = YAML::Load(config_yaml_);
            std::vector<double> ori{map_origin_[0], map_origin_[1], map_origin_[2]};
            yaml["origin"] = ori;
            std::ofstream fout(config_yaml_);
            fout << yaml;
            break;
        }
    }
    if (origin_set) {
        LOG(INFO) << "removing origin from rtk";
        for (auto& gnss : gnss_datas_) {
            gnss.second->utm_pose_.translation() -= map_origin_;
        }
    }
}

void Frontend::ExtractKeyFrame(const NavStated& state) {
    if (last_kf_ == nullptr) {
        if (!lio_->GetCurrentScan()) {
            // 未初始化完成
            return;
        }
        // 构建第一帧keyframe 传入世界位姿和cloud
        auto kf = std::make_shared<Keyframe>(state.timestamped_, kf_id_++, state.GetSE3(), lio_->GetCurrentScan());
        FindGPSPose(kf);
        kf->SaveAndUnloadScan("./data/ch9/");
        keyframes_.emplace(kf->id_, kf);
        last_kf_ = kf;
    } else {
        // 根据距离提取关键帧
        SE3 delta = last_kf_->lidar_pose_.inverse() * state.GetSE3();
        if (delta.translation().norm() > kf_distance_th_ ||
            delta.so3().log().norm() > kf_angular_th_ * math::kDEG2RAD) {
            auto kf = std::make_shared<Keyframe>(state.timestamped_, kf_id_++, state.GetSE3(), lio_->GetCurrentScan());
            FindGPSPose(kf);
            kf->SaveAndUnloadScan("./data/ch9/");
            keyframes_.emplace(kf->id_, kf);
            LOG(INFO) << "生成关键帧" << kf->id_;
            last_kf_ = kf;
        }
    }
}

/// 确定某个关键帧的GPS pose
void Frontend::FindGPSPose(std::shared_ptr<Keyframe> kf) {
    SE3 pose;
    GNSSPtr match;
    if (math::PoseInterp<GNSSPtr>(kf->timestamped_, gnss_datas_,
                                  [](const GNSSPtr& gnss) -> SE3 { return gnss->utm_pose_; }, pose, match)) {
        kf->rtk_pose_ = pose;
        kf->rtk_valid_ = true;
    } else {
        kf->rtk_valid_ = false;
    }
}

/// 保存各个关键帧位姿，点云在生成时已经保存
void Frontend::SaveKeyframes() {
    // 生成所有的keyframe
    std::ofstream fout("./data/ch9/keyframes.txt");
    for (auto& kfp : keyframes_) {
        kfp.second->Save(fout);
    }
    fout.close();
}

}  // namespace slam_learn::mapping