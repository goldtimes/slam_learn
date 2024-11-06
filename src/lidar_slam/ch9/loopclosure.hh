#pragma once

#include <map>
#include <vector>
#include "common/eigen_types.hh"
#include "common/sensors//point_type.hh"
#include "lidar_slam/ch9/keyframe.hh"

namespace slam_learn::mapping {
// 候选的回环检测帧
struct LoopCandidate {
    LoopCandidate() {
    }
    LoopCandidate(unsigned int id1, unsigned int id2, SE3 Tij) : idx1_(id1), idx2_(id2), Tij_(Tij) {
    }

    IdType idx1_ = 0;
    IdType idx2_ = 0;
    SE3 Tij_;
    double ndt_score_ = 0.0;
};

class LoopClosure {
   public:
    explicit LoopClosure(const std ::string& config_yaml);
    bool Init();
    void Run();

   private:
    /// 提取回环的候选
    void DetectLoopCandidates();

    /// 计算回环候选帧的相对运动
    void ComputeLoopCandidates();

    /// 计算一个回环候选是否成立
    void ComputeForCandidate(LoopCandidate& c);

    /// 保存计算结果
    void SaveResults();

   private:
    std::vector<LoopCandidate> loop_candiates_;
    // 被选为关键帧之间的id差
    int min_id_interval_ = 50;
    // 候选帧之间的最小距离
    double min_distance_ = 30;
    // 选择了一个候选帧，隔开多少个id之后再选
    int skip_id_ = 5;
    // 有效的回环分数阈值
    double ndt_score_th_ = 2.5;

    std::map<IdType, KeyFramePtr> keyframes_;
    std::string yaml_;
};
}  // namespace slam_learn::mapping
