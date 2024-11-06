#pragma once
#include "lidar_slam/ch9/optimization.hh"

#include "common/eigen_types.hh"
#include "common/g2o_types.hh"
#include "common/math_utils.hh"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/robust_kernel_impl.h"
#include "lidar_slam/ch9/keyframe.hh"
#include "lidar_slam/ch9/loopclosure.hh"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <boost/format.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace slam_learn::mapping {

/// 打印优化信息
template <typename T>
std::string print_info(const std::vector<T>& edges, double th) {
    std::vector<double> chi2;
    for (auto& edge : edges) {
        if (edge->level() == 0) {
            // 计算误差
            edge->computeError();
            chi2.push_back(edge->chi2());
        }
    }

    std::sort(chi2.begin(), chi2.end());
    double ave_chi2 = std::accumulate(chi2.begin(), chi2.end(), 0.0) / chi2.size();
    boost::format fmt("数量: %d, 均值: %f, 中位数: %f, 0.1分位: %f, 0.9分位: %f, 0.95分位：%f, 最大值: %f, 阈值: %f\n");
    if (!chi2.empty()) {
        std::string str = (fmt % chi2.size() % ave_chi2 % chi2[chi2.size() / 2] % chi2[int(chi2.size() * 0.1)] %
                           chi2[int(chi2.size() * 0.9)] % chi2[int(chi2.size() * 0.95)] % chi2.back() % th)
                              .str();
        return str;
    }
    return std::string();
}

Optimization::Optimization(const std::string& yaml) {
    yaml_ = yaml;
}

/// 初始化，指定为第一轮还是第二轮
bool Optimization::Init(int stage) {
    stage_ = stage;
    // 加载所有的keyframe
    if (!LoadKeyFrames("./data/ch9/keyframes.txt", keyframes_)) {
        LOG(ERROR) << "cannot load keyframes.txt";
        return false;
    }
    LOG(INFO) << "keyframes: " << keyframes_.size();
    // 读参数
    auto yaml = YAML::LoadFile(yaml_);
    rtk_outlier_th_ = yaml["rtk_outlier_th"].as<double>();
    lidar_continuous_num_ = yaml["lidar_continuous_num"].as<int>();
    rtk_has_rot_ = yaml["rtk_has_rot"].as<bool>();

    rtk_pos_noise_ = yaml["rtk_pos_noise"].as<double>();
    rtk_ang_noise_ = yaml["rtk_ang_noise"].as<double>() * math::kDEG2RAD;
    rtk_height_noise_ratio_ = yaml["rtk_height_noise_ratio"].as<double>();

    std::vector<double> rtk_ext_t = yaml["rtk_ext"]["t"].as<std::vector<double>>();
    TBG_ = SE3(SO3(), Vec3d(rtk_ext_t[0], rtk_ext_t[1], rtk_ext_t[2]));
    LOG(INFO) << "TBG = \n" << TBG_.matrix();
    // 二阶段才优化回环检测，第一轮是利用
    if (stage_ == 2) {
        // 加载回环的信息
        LoadLoopCandidates();
    }
    return true;
}

void Optimization::LoadLoopCandidates() {
    std::ifstream fin("./data/ch9/loops.txt");
    if (!fin.is_open()) {
        LOG(WARNING) << "cannot load file: ./data/ch9/loops.txt";
        return;
    }
    auto load_SE3 = [](std::istream& fin) {
        SE3 ret;
        double q[4];
        double t[3];
        fin >> t[0] >> t[1] >> t[2] >> q[0] >> q[1] >> q[2] >> q[3];
        return SE3(Quatd(q[3], q[0], q[1], q[2]), Vec3d(t[0], t[1], t[2]));
    };

    // 遍历文件
    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            break;
        }
        std::stringstream ss;
        ss << line;
        LoopCandidate lc;
        ss >> lc.idx1_ >> lc.idx2_ >> lc.ndt_score_;
        lc.Tij_ = load_SE3(ss);
        loop_candidates_.emplace_back(lc);
    }
    LOG(INFO) << "loaded loops: " << loop_candidates_.size();
}

/// 执行优化
void Optimization::Run() {
    LOG(INFO) << "running optimization on stage " << stage_;
    if (!rtk_has_rot_ && stage_ == 1) {
        InitialAlign();
    }
    BuildProblem();  // 建立问题

    SaveG2O("./data/ch9/before.g2o");
    LOG(INFO) << "RTK 误差：" << print_info(gnss_edge_, rtk_outlier_th_);
    LOG(INFO) << "RTK 平移误差：" << print_info(gnss_trans_edge_, rtk_outlier_th_);
    LOG(INFO) << "lidar 误差：" << print_info(lidar_odom_edge_, 0);

    Solve();           // 带着RTK求解一遍
    RemoveOutliers();  // 移除异常值
    Solve();           // 再求解一遍

    SaveG2O("./data/ch9/after.g2o");

    SaveResults();  // 保存结果
    LOG(INFO) << "done";
}

void Optimization::InitialAlign() {
    // 取出lidar_pose
    // 取出rtk_pose
    std::vector<Vec3d> lidar_pts, rtk_pts;
    for (const auto& kfp : keyframes_) {
        lidar_pts.emplace_back(kfp.second->lidar_pose_.translation());
        rtk_pts.emplace_back(kfp.second->rtk_pose_.translation());
    }
    Vec3d lidar_center;
    Vec3d rtk_center;
    int lidar_pose_size = lidar_pts.size();
    for (int i = 0; i < lidar_pose_size; ++i) {
        lidar_center += lidar_pts[i];
        rtk_center += rtk_pts[i];
    }
    lidar_center /= lidar_pose_size;
    rtk_center /= lidar_pose_size;
    LOG(INFO) << "lidar_center: " << lidar_center.transpose() << ", rtk_center: " << rtk_center.transpose();
    std::vector<Vec3d> lidar_rm_center_pts;
    std::vector<Vec3d> rtk_rm_center_pts;
    // 所有点remove center
    for (int i = 0; i < lidar_pose_size; ++i) {
        lidar_rm_center_pts[i] = lidar_pts[i] - lidar_center;
        rtk_rm_center_pts[i] = rtk_pts[i] - rtk_center;
    }
    // 计算每个点的q1*q2^T
    Mat3d w = Mat3d::Identity();
    for (int i = 0; i < lidar_pose_size; ++i) {
        w += lidar_rm_center_pts[i] * rtk_rm_center_pts[i].transpose();
    }
    // svd求解
    Eigen::JacobiSVD<Mat3d> svd(w, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3d U = svd.matrixU();
    Mat3d V = svd.matrixV();
    Mat3d R = U * (V.transpose());
    if (R.determinant() < 0) {
        R = -R;
    }
    Vec3d t = lidar_center - R * rtk_center;

    // change lidar pose
    SE3 T(R, t);
    LOG(INFO) << "initial trans: \n" << T.matrix();
    for (auto& kfp : keyframes_) {
        kfp.second->lidar_pose_ = T * kfp.second->lidar_pose_;
    }
}

void Optimization::BuildProblem() {
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer_.setAlgorithm(solver);
    // 第一阶段主要使用关键帧的运动和rtk的观测来优化
    // 第二阶段主要使用关键帧的运动和rtk的观测+loopclosure来优化
    AddVertices();
    AddRTKEdges();
    AddLidarEdges();
    AddLoopEdges();
}

void Optimization::AddVertices() {
    for (auto& kfp : keyframes_) {
        auto kf = kfp.second;
        VertexPose* vertex = new VertexPose();
        vertex->setId(kf->id_);

        if (stage_ == 1) {
            // 阶段为1，输入原始的pose
            vertex->setEstimate(kf->lidar_pose_);
        } else {
            // 阶段为2,输入阶段1得到的pose
            vertex->setEstimate(kf->opti_pose_1_);
        }
        optimizer_.addVertex(vertex);
        vertices_.emplace(kf->id_, vertex);
    }
    LOG(INFO) << "vertex: " << vertices_.size();
}
void Optimization::AddRTKEdges() {
    /// RTK 噪声设置
    Mat3d info_pos = Mat3d::Identity() * 1.0 / (rtk_pos_noise_ * rtk_pos_noise_);
    info_pos(2, 2) = 1.0 / (rtk_height_noise_ratio_ * rtk_pos_noise_ * rtk_height_noise_ratio_ * rtk_pos_noise_);
    Mat3d info_ang = Mat3d::Identity() * 1.0 / (rtk_ang_noise_ * rtk_ang_noise_);
    Mat6d info_all = Mat6d::Identity();
    info_all.block<3, 3>(0, 0) = info_ang;
    info_all.block<3, 3>(3, 3) = info_pos;

    LOG(INFO) << "Info of rtk trans: " << info_pos.diagonal().transpose();

    if (stage_ == 2) {
        info_pos *= 0.01;
        info_all *= 0.01;
    }
    for (auto& kfp : keyframes_) {
        auto kf = kfp.second;
        if (!kf->rtk_valid_) {
            continue;
        }
        if (kf->rtk_heading_valid_) {
            auto gnss_edge = new EdgeGNSS(vertices_.at(kf->id_), kf->rtk_pose_);
            gnss_edge->setInformation(info_all);
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(rtk_outlier_th_);
            gnss_edge->setRobustKernel(rk);
            optimizer_.addEdge(gnss_edge);
            gnss_edge_.emplace_back(gnss_edge);
        } else {
            auto edge = new EdgeGNSSTransOnly(vertices_.at(kf->id_), kf->rtk_pose_.translation(), TBG_);
            edge->setInformation(info_pos);
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(rtk_outlier_th_);
            edge->setRobustKernel(rk);
            optimizer_.addEdge(edge);
            gnss_trans_edge_.emplace_back(edge);
        }
    }
    LOG(INFO) << "gnss edges: " << gnss_edge_.size() << ", " << gnss_trans_edge_.size();
}
void Optimization::AddLidarEdges() {
    // 添加两关键帧之间的约束
    double lidar_pose_noise = 0.01;
    double lidar_angular_noise = 0.1 * math::kDEG2RAD;
    Mat3d info_pos = Mat3d::Identity() * 1.0 / (lidar_pose_noise * lidar_pose_noise);
    Mat3d info_ang = Mat3d::Identity() * 1.0 / (lidar_angular_noise * lidar_angular_noise);
    Mat6d info_all = Mat6d::Identity();

    info_all.block<3, 3>(0, 0) = info_pos;
    info_all.block<3, 3>(3, 3) = info_ang;
    // 当前帧和接下来三帧的相对运动
    for (auto iter = keyframes_.begin(); iter != keyframes_.end(); ++iter) {
        auto iter_next = iter;
        for (int i = 0; i < lidar_continuous_num_; ++i) {
            iter_next++;
            if (iter_next == keyframes_.end()) {
                break;
            }
            EdgeRelativeMotion* edge =
                new EdgeRelativeMotion(vertices_.at(iter->second->id_), vertices_.at(iter_next->second->id_),
                                       iter->second->lidar_pose_.inverse() * iter_next->second->lidar_pose_);
            edge->setInformation(info_all);
            optimizer_.addEdge(edge);
            lidar_odom_edge_.emplace_back(edge);
        }
    }
    LOG(INFO) << "lidar edges: " << lidar_odom_edge_.size();
}
void Optimization::AddLoopEdges() {
    if (stage_ == 1) {
        return;
    }

    const double loop_pos_noise = 0.1, loop_ang_noise = 0.5 * math::kDEG2RAD;
    Mat3d info_pos = Mat3d::Identity() * 1.0 / (loop_pos_noise * loop_pos_noise);
    Mat3d info_ang = Mat3d::Identity() * 1.0 / (loop_ang_noise * loop_ang_noise);
    Mat6d info_all = Mat6d::Identity();
    info_all.block<3, 3>(0, 0) = info_pos;
    info_all.block<3, 3>(3, 3) = info_ang;

    const double loop_rk_th = 5.2;
    for (const auto& lc : loop_candidates_) {
        auto edge = new EdgeRelativeMotion(vertices_.at(lc.idx1_), vertices_.at(lc.idx2_), lc.Tij_);
        edge->setInformation(info_all);
        auto rk = new g2o::RobustKernelCauchy();
        rk->setDelta(loop_rk_th);
        edge->setRobustKernel(rk);
        optimizer_.addEdge(edge);
        loop_edge_.emplace_back(edge);
    }
}

void Optimization::Solve() {
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization(0);
    optimizer_.optimize(100);

    LOG(INFO) << "RTK 误差：" << print_info(gnss_edge_, rtk_outlier_th_);
    LOG(INFO) << "RTK 平移误差：" << print_info(gnss_trans_edge_, rtk_outlier_th_);
    LOG(INFO) << "lidar 误差：" << print_info(lidar_odom_edge_, 0);
    LOG(INFO) << "loop 误差：" << print_info(loop_edge_, 0);
}
/**
 * @brief 去除gnss异常值 取出 loop的异常值
 */
void Optimization::RemoveOutliers() {
    int cnt_outlier_removed = 0;
    auto remove_outlier = [&cnt_outlier_removed](g2o::OptimizableGraph::Edge* e) {
        // gps和lidar_world的位姿误差>设置的阈值
        if (e->chi2() > e->robustKernel()->delta()) {
            e->setLevel(1);
            cnt_outlier_removed++;
        } else {
            e->setRobustKernel(nullptr);
        }
    };

    std::for_each(gnss_edge_.begin(), gnss_edge_.end(), remove_outlier);
    std::for_each(gnss_trans_edge_.begin(), gnss_trans_edge_.end(), remove_outlier);
    LOG(INFO) << "gnss outlier: " << cnt_outlier_removed << "/" << gnss_edge_.size() + gnss_trans_edge_.size();

    cnt_outlier_removed = 0;
    std::for_each(loop_edge_.begin(), loop_edge_.end(), remove_outlier);
    LOG(INFO) << "loop outlier: " << cnt_outlier_removed << "/" << loop_edge_.size();
}

void Optimization::SaveResults() {
    for (auto& v : vertices_) {
        if (stage_ == 1) {
            keyframes_.at(v.first)->opti_pose_1_ = v.second->estimate();
        } else {
            keyframes_.at(v.first)->opti_pose_2_ = v.second->estimate();
        }
    }
    // 比较优化后和rtk的误差
    std::vector<double> rtk_trans_error;
    for (auto& kfp : keyframes_) {
        auto kf = kfp.second;
        Vec3d tWG = kf->rtk_pose_.translation();
        Vec3d t_opti = (kf->opti_pose_1_ * TBG_).translation();
        double n = (tWG - t_opti).head<2>().norm();
        rtk_trans_error.emplace_back(n);
    }
    std::sort(rtk_trans_error.begin(), rtk_trans_error.end());
    LOG(INFO) << "med error: " << rtk_trans_error[rtk_trans_error.size() / 2];

    // 写入文件
    system("rm ./data/ch9/keyframes.txt");
    std::ofstream fout("./data/ch9/keyframes.txt");
    for (auto& kfp : keyframes_) {
        kfp.second->Save(fout);
    }
    fout.close();
}

void Optimization::SaveG2O(const std::string& file_name) {
    std::ofstream fout(file_name);
    for (auto& v : vertices_) {
        v.second->write(fout);
    }

    for (auto& e : lidar_odom_edge_) {
        e->write(fout);
    }
    for (auto& e : loop_edge_) {
        e->write(fout);
    }
    fout.close();
}
}  // namespace slam_learn::mapping