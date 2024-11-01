#pragma once

#include <glog/logging.h>
#include <deque>
#include <memory>
#include <queue>
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

    bool IsLeaf() const {
        return left_ == nullptr && right_ == nullptr;
    }  // 是否为叶子
};

// 存放knn的结果
struct NodeAndDistance {
    NodeAndDistance(KdTreeNode* node, float dis2) : node_(node), distance2_(dis2) {
    }

    KdTreeNode* node_ = nullptr;
    float distance2_ = 0;

    bool operator<(const NodeAndDistance& other) const {
        return distance2_ < other.distance2_;
    }
};

class KdTree {
   public:
    explicit KdTree() = default;
    ~KdTree() {
        Clear();
    }
    bool BuildTree(CloudPtr cloud);

   public:
    void Insert(const std::vector<int>& points, KdTreeNode* node);
    /// 打印所有节点信息
    void PrintAll();

    void SetEnableANN(bool use_ann = true, float alpha = 0.1) {
        approximate_ = use_ann;
        alpha_ = alpha;
    }

    /// 获取k最近邻
    bool GetClosestPoint(const PointXYZI& pt, std::vector<int>& closest_idx, int k = 5);

    /// 并行为点云寻找最近邻
    bool GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);

    /// 返回节点数量
    size_t size() const {
        return leaf_node_size_;
    }

   private:
    void Reset();
    void Clear();

    bool FindSplitAxisAndThresh(const std::vector<int>& points, int& axis, float& th, std::vector<int>& left,
                                std::vector<int>& right);

    // Knn 相关
    /**
     * 检查给定点在kdtree node上的knn，可以递归调用
     * @param pt     查询点
     * @param node   kdtree 节点
     */
    void Knn(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /**
     * 对叶子节点，计算它和查询点的距离，尝试放入结果中
     * @param pt    查询点
     * @param node  Kdtree 节点
     */
    void ComputeDisForLeaf(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    bool NeedExpand(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const;

   private:
    // 根节点
    std::shared_ptr<KdTreeNode> root_ = nullptr;

    std::unordered_map<int, KdTreeNode*> nodes_record_;
    // 点云
    std::vector<Vec3f> cloud_;
    size_t leaf_node_size_ = 0;
    int tree_node_id_ = 0;
    // 最邻近值
    int k_;
    // 近似邻近
    bool approximate_ = true;
    float alpha_ = 0.1;
};
}  // namespace slam_learn::kdtree