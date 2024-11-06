#pragma once
#include <g2o/core/sparse_optimizer.h>
#include <map>
#include <vector>
#include "common/eigen_types.hh"
#include "common/g2o_types.hh"
#include "common/math_utils.hh"
#include "lidar_slam/ch9/keyframe.hh"
#include "lidar_slam/ch9/loopclosure.hh"

namespace slam_learn::mapping {

class Optimization {
   public:
    explicit Optimization(const std::string& yaml);

    /// 初始化，指定为第一轮还是第二轮
    bool Init(int stage = 1);

    /// 执行优化
    void Run();

   private:
    /// 如果RTK全程不带旋转，那么先对激光和RTK做一次ICP来对齐整条轨迹
    void InitialAlign();

    /// 构建优化问题
    void BuildProblem();

    /// 添加顶点
    void AddVertices();

    /// 添加各种位姿观测
    void AddRTKEdges();
    void AddLidarEdges();
    void AddLoopEdges();

    /// 求解问题
    void Solve();

    /// 移除异常值
    void RemoveOutliers();

    /// 保存优化结果
    void SaveResults();

    /// 读取回环候选
    void LoadLoopCandidates();

    /// 保存g2o文件
    void SaveG2O(const std::string& file_name);

   private:
    std::string yaml_;  // 配置文件
    std::map<IdType, KeyFramePtr> keyframes_;

    bool rtk_has_rot_ = false;
    // 第一阶段的优化
    int stage_ = 1;
    SE3 TBG_;  // body->GNSS的外参

    std::vector<LoopCandidate> loop_candidates_;
    // g2o顶点
    std::map<IdType, VertexPose*> vertices_;
    // 优化器
    g2o::SparseOptimizer optimizer_;
    // g2o边
    std::vector<EdgeGNSS*> gnss_edge_;
    std::vector<EdgeGNSSTransOnly*> gnss_trans_edge_;

    std::vector<EdgeRelativeMotion*> lidar_odom_edge_;
    std::vector<EdgeRelativeMotion*> loop_edge_;
    // 参数
    double rtk_outlier_th_ = 1.0;   // RTK 异常阈值
    int lidar_continuous_num_ = 3;  // 雷达相邻位姿数量
    double rtk_pos_noise_ = 0.5;
    double rtk_ang_noise_ = 2.0 * math::kDEG2RAD;
    double rtk_height_noise_ratio_ = 20.0;
};
}  // namespace slam_learn::mapping
