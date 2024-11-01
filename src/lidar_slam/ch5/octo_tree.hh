#pragma once

#include <glog/logging.h>
#include <cassert>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>
#include "common/eigen_types.hh"
#include "common/sensors/point_type.hh"

namespace slam_learn::octo_tree {

struct Box3D {
    Box3D() = default;
    Box3D(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
        : min_{min_x, min_y, min_z}, max_{max_x, max_y, max_z} {
    }

    /// 判断pt是否在内部
    bool Inside(const Vec3f& pt) const {
        return pt[0] <= max_[0] && pt[0] >= min_[0] && pt[1] <= max_[1] && pt[1] >= min_[1] && pt[2] <= max_[2] &&
               pt[2] >= min_[2];
    }
    // 外侧点到边界的最大距离
    // 暂时还不理解
    float Dist(const Vec3f& pt) const {
        // 距离那个平面最近的距离
        float ret = 0;
        // 依次比较x,y,z轴，每个轴都有两种情况，想想一个点,可以再正方体的任何方向
        for (int i = 0; i < 3; ++i) {
            if (pt[i] < min_[i]) {
                float d = min_[i] - pt[i];
                // 遍历得到距离哪个轴的最小值
                ret = d > ret ? d : ret;
            } else if (pt[i] > max_[i]) {
                float d = pt[i] - max_[i];
                ret = d > ret ? d : ret;
            }
        }
        assert(ret >= 0);
        return ret;
    }

    float min_[3] = {0, 0, 0};
    float max_[3] = {0, 0, 0};
};

struct OctoTreeNode {
    // 节点id
    int node_id_ = -1;
    // 点的下标
    int point_idx_ = -1;
    // 边界框
    Box3D box_;
    OctoTreeNode* children[9] = {nullptr};
    bool IsLeaf() const {
        for (const OctoTreeNode* node : children) {
            if (node != nullptr) {
                return false;
            }
        }
        return true;
    }
};

struct NodeAndDistance {
    NodeAndDistance(OctoTreeNode* node, float dis2) : node_(node), distance_(dis2) {
    }

    OctoTreeNode* node_;
    float distance_ = 0;
    bool operator<(const NodeAndDistance& other) const {
        return distance_ < other.distance_;
    }
};

class OctoTree {
   public:
    OctoTree() = default;
    ~OctoTree() {
        Clear();
    }

    bool GetClosestPoint(const PointXYZI& pt, std::vector<int>& closest_indx, int k = 5) const;
    bool GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);

    bool BuildTree(const CloudPtr cloud);
    /// 设置近似最近邻参数
    void SetApproximate(bool use_ann = true, float alpha = 0.1) {
        approximate_ = use_ann;
        alpha_ = alpha;
    }
    /// 返回节点数量
    size_t size() const {
        return leaf_node_size_;
    }

   private:
    void Clear();
    void Reset();

    /**
     * 在node处插入点
     * @param points
     * @param node
     */
    void Insert(const IndexVec& points, OctoTreeNode* node);

    /// 为全局点云生成边界框
    Box3D ComputeBoundingBox();

    /**
     * 展开一个节点
     * @param [in] node 被展开的节点
     * @param [in] parent_idx 父节点的点云索引
     * @param [out] children_idx 子节点的点云索引
     */
    void ExpandNode(OctoTreeNode* node, const IndexVec& parent_idx, std::vector<IndexVec>& children_idx);

    // Knn 相关
    /**
     * 检查给定点在kdtree node上的knn，可以递归调用
     * @param pt     查询点
     * @param node   kdtree 节点
     */
    void Knn(const Vec3f& pt, OctoTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /**
     * 对叶子节点，计算它和查询点的距离，尝试放入结果中
     * @param pt    查询点
     * @param node  Kdtree 节点
     */
    void ComputeDisForLeaf(const Vec3f& pt, OctoTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /**
     * 检查node下是否需要展开
     * @param pt   查询点
     * @param node Kdtree 节点
     * @return true if 需要展开
     */
    bool NeedExpand(const Vec3f& pt, OctoTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const;

   private:
    // 记录节点
    std::unordered_map<int, OctoTreeNode*> nodes_record_;
    // node id
    int tree_node_id_ = 0;
    //  叶子节点的个数
    size_t leaf_node_size_ = 0;
    int k_ = 5;
    // 根节点
    std::shared_ptr<OctoTreeNode> root_ = nullptr;
    std::vector<Vec3f> cloud_;
    // flann
    bool approximate_ = true;
    float alpha_ = 1.0;
};
}  // namespace slam_learn::octo_tree