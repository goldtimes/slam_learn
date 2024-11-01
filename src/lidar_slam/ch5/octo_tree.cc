#include "lidar_slam/ch5/octo_tree.hh"
#include <algorithm>
#include <cstddef>
#include <execution>
#include <limits>
#include <queue>
#include <vector>
#include "common/lidar_utils.hh"
#include "common/math_utils.hh"
namespace slam_learn::octo_tree {

bool OctoTree::BuildTree(const CloudPtr cloud) {
    if (cloud->empty()) {
        return false;
    }
    cloud_.clear();
    cloud_.resize(cloud->size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud_[i] = lidar_utils::ToVec3f(cloud->points[i]);
    }
    // 点云转换为eigen
    // std::transform(cloud->points.begin(), cloud->points.end(), cloud_.begin(),
    //                [](const auto& pt) { return lidar_utils::ToVec3f(pt); });
    Clear();
    Reset();
    // idx
    std::vector<int> idx(cloud->size());
    for (int i = 0; i < cloud->points.size(); ++i) {
        idx[i] = i;
    }
    // std::for_each(idx.begin(), idx.end(), [i = 0](int& id) mutable { id = i++; });
    // 计算了整个点云的3d Box
    root_->box_ = ComputeBoundingBox();
    // 插入节点
    Insert(idx, root_.get());
    return true;
}

void OctoTree::Insert(const IndexVec& points, OctoTreeNode* node) {
    // 处理根节点
    nodes_record_.insert({node->node_id_, node});
    if (points.empty()) {
        return;
    }
    if (points.size() == 1) {
        node->point_idx_ = points[0];
        leaf_node_size_++;
        return;
    }
    // 点数不为1,就向下展开
    std::vector<std::vector<int>> children_points;
    // 想象一个很大的框先包住点云，然后先切割为8个立方体。然后再每个立方体中在切割8个立方体，这样对点云空间划分
    ExpandNode(node, points, children_points);
    // 递归调用insert
    for (int i = 0; i < 8; ++i) {
        Insert(children_points[i], node->children[i]);
    }
}

/**
 *@brief std::vector<IndexVec>& children_idx 8个立方体中，每个立方体都有多个点云
 */
void OctoTree::ExpandNode(OctoTreeNode* node, const IndexVec& parent_idx, std::vector<IndexVec>& children_idx) {
    children_idx.resize(8);
    for (int i = 0; i < 8; ++i) {
        // 分配子节点
        node->children[i] = new OctoTreeNode();
        // 子节点的id累加
        node->children[i]->node_id_ = tree_node_id_++;
    }
    // 当前节点的box
    const Box3D& b = node->box_;
    // 中心点
    float c_x = 0.5 * (node->box_.min_[0] + node->box_.max_[0]);
    float c_y = 0.5 * (node->box_.min_[1] + node->box_.max_[1]);
    float c_z = 0.5 * (node->box_.min_[2] + node->box_.max_[2]);

    // 8个外框示意图
    // clang-format off
    // 第一层：左上1 右上2 左下3 右下4
    // 第二层：左上5 右上6 左下7 右下8
    //     ---> x    /-------/-------/|
    //    /|        /-------/-------/||
    //   / |       /-------/-------/ ||
    //  y  |z      |       |       | /|
    //             |_______|_______|/|/
    //             |       |       | /
    //             |_______|_______|/
    // clang-format on

    node->children[0]->box_ = {b.min_[0], c_x, b.min_[1], c_y, b.min_[2], c_z};
    node->children[1]->box_ = {c_x, b.max_[0], b.min_[1], c_y, b.min_[2], c_z};
    node->children[2]->box_ = {b.min_[0], c_x, c_y, b.max_[1], b.min_[2], c_z};
    node->children[3]->box_ = {c_x, b.max_[0], c_y, b.max_[1], b.min_[2], c_z};

    node->children[4]->box_ = {b.min_[0], c_x, b.min_[1], c_y, c_z, b.max_[2]};
    node->children[5]->box_ = {c_x, b.max_[0], b.min_[1], c_y, c_z, b.max_[2]};
    node->children[6]->box_ = {b.min_[0], c_x, c_y, b.max_[1], c_z, b.max_[2]};
    node->children[7]->box_ = {c_x, b.max_[0], c_y, b.max_[1], c_z, b.max_[2]};

    // 判断点云在那个立方体中
    for (const auto& id : parent_idx) {
        const auto& pt = cloud_[id];
        // 遍历所有的立方体
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]->box_.Inside(pt)) {
                children_idx[i].emplace_back(id);
                break;
            }
        }
    }
}

/// 为全局点云生成边界框
Box3D OctoTree::ComputeBoundingBox() {
    float min_values[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max()};
    float max_values[3] = {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                           -std::numeric_limits<float>::max()};
    for (const auto& pt : cloud_) {
        for (int i = 0; i < 3; ++i) {
            max_values[i] = pt[i] > max_values[i] ? pt[i] : max_values[i];
            min_values[i] = pt[i] < min_values[i] ? pt[i] : min_values[i];
        }
    }

    return Box3D(min_values[0], max_values[0], min_values[1], max_values[1], min_values[2], max_values[2]);
}

void OctoTree::Reset() {
    tree_node_id_ = 0;
    root_.reset(new OctoTreeNode());
    root_->node_id_ = tree_node_id_++;
    leaf_node_size_ = 0;
}

void OctoTree::Clear() {
    for (const auto& np : nodes_record_) {
        if (np.second != root_.get()) {
            delete np.second;
        }
    }

    root_ = nullptr;
    leaf_node_size_ = 0;
    tree_node_id_ = 0;
}

bool OctoTree::GetClosestPoint(const PointXYZI& pt, std::vector<int>& closest_idx, int k) const {
    if (k > leaf_node_size_) {
        LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << leaf_node_size_;
        return false;
    }
    std::priority_queue<NodeAndDistance> knn_result;
    Knn(lidar_utils::ToVec3f(pt), root_.get(), knn_result);

    // 根据返回的结果排序
    closest_idx.resize(knn_result.size());
    for (int i = closest_idx.size() - 1; i >= 0; --i) {
        // 倒序插入
        closest_idx[i] = knn_result.top().node_->point_idx_;
        knn_result.pop();
    }
    return true;
}

bool OctoTree::GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k) {
    k_ = k;
    matches.resize(cloud->size() * k);
    // 索引
    std::vector<int> index(cloud->size());
    for (int i = 0; i < cloud->points.size(); ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &cloud, &matches, &k](int idx) {
        std::vector<int> closest_idx;
        GetClosestPoint(cloud->points[idx], closest_idx, k);

        for (int i = 0; i < k; ++i) {
            matches[idx * k + i].second = idx;
            if (i < closest_idx.size()) {
                matches[idx * k + i].first = closest_idx[i];
            } else {
                matches[idx * k + i].first = math::kINVALID_ID;
            }
        }
    });

    return true;
}

void OctoTree::Knn(const Vec3f& pt, OctoTreeNode* node, std::priority_queue<NodeAndDistance>& result) const {
    if (node->IsLeaf()) {
        if (node->point_idx_ != -1) {
            ComputeDisForLeaf(pt, node, result);
            return;
        }
        return;
    }
    //检查查询的点在哪个格子中
    int idx_child = -1;
    float min_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < 8; ++i) {
        if (node->children[i]->box_.Inside(pt)) {
            idx_child = i;
            break;
        }
        // 计算它到哪个box最近
        else {
            float d = node->children[i]->box_.Dist(pt);
            if (d < min_dist) {
                min_dist = d;
                idx_child = i;
            }
        }
    }
    // 获得最近的立方体和距离它的距离
    // 递归调用
    Knn(pt, node->children[idx_child], result);
    // 检查父节点的其他孩子节点
    for (int i = 0; i < 8; ++i) {
        if (i == idx_child) {
            continue;
        }
        if (NeedExpand(pt, node->children[i], result)) {
            Knn(pt, node->children[i], result);
        }
    }
}

/**
 * 对叶子节点，计算它和查询点的距离，尝试放入结果中
 * @param pt    查询点
 * @param node  Kdtree 节点
 */
void OctoTree::ComputeDisForLeaf(const Vec3f& pt, OctoTreeNode* node,
                                 std::priority_queue<NodeAndDistance>& result) const {
    float dist2 = (pt - cloud_[node->point_idx_]).squaredNorm();
    if (result.size() < k_) {
        result.push({node, dist2});
    } else {
        if (dist2 < result.top().distance_) {
            result.push({node, dist2});
            result.pop();
        }
    }
}

/**
 * 检查node下是否需要展开
 * @param pt   查询点
 * @param node Kdtree 节点
 * @return true if 需要展开
 */
bool OctoTree::NeedExpand(const Vec3f& pt, OctoTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const {
    if (knn_result.size() < k_) {
        return true;
    }
    if (approximate_) {
        float d = node->box_.Dist(pt);
        if ((d * d) < knn_result.top().distance_ * alpha_) {
            return true;
        } else {
            return false;
        }
    } else {
        // 不用flann时，按通常情况查找
        float d = node->box_.Dist(pt);
        if ((d * d) < knn_result.top().distance_) {
            return true;
        } else {
            return false;
        }
    }
}
}  // namespace slam_learn::octo_tree