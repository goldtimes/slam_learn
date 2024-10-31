#pragma once

#include <glog/logging.h>
#include <deque>
#include <memory>
#include <unordered_map>
#include "common/eigen_types.hh"
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
#include "common/sensors/point_type.hh"

namespace slam_learn::kdtree {
// 树结构体
struct KdTreeNode {
    int id_ = -1;
    int point_idx_ = 0;            // 点的索引
    int axis_index_ = 0;           // 分割轴
    float split_thresh_ = 0.0;     //分割的阈值
    KdTreeNode* left_ = nullptr;   // 左子树
    KdTreeNode* right_ = nullptr;  // 右子树
};

class KdTree {
   public:
    explicit KdTree() = default;
    bool BuildTree(CloudPtr cloud);

   public:
    void Insert(const std::vector<int>& points, KdTreeNode* node);
    /// 打印所有节点信息
    void PrintAll();

   private:
    void Reset();
    void Clear();

    bool FindSplitAxisAndThresh(const std::vector<int>& points, int& axis, float& th, std::vector<int>& left,
                                std::vector<int>& right);

   private:
    // 根节点
    std::shared_ptr<KdTreeNode> root_ = nullptr;

    std::unordered_map<int, KdTreeNode*> nodes_record_;
    // 点云
    std::vector<Vec3f> cloud_;
    size_t leaf_node_size_ = 0;
    int tree_node_id_ = 0;
};
}  // namespace slam_learn::kdtree