#include "lidar_slam/ch5/kdtree.hh"

namespace slam_learn::kdtree {

bool KdTree::BuildTree(CloudPtr cloud) {
    if (cloud->empty()) {
        return false;
    }
    cloud_.clear();
    cloud_.resize(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        cloud_[i] = lidar_utils::ToVec3f(cloud->points[i]);
    }
    Clear();
    Reset();

    std::vector<int> idx(cloud->size());
    std::for_each(idx.begin(), idx.end(), [id = 0](int& i) mutable {
        i = id++;
        LOG(INFO) << "id:" << id;
    });
    // 传入所有点云的下标，传入根节点
    Insert(idx, root_.get());
    return true;
}

void KdTree::Insert(const std::vector<int>& points, KdTreeNode* node) {
    // 记录该节点
    nodes_record_.insert({node->id_, node});
    if (points.empty()) {
        return;
    }
    // 退出条件
    if (points.size() == 1) {
        leaf_node_size_++;
        node->point_idx_ = points[0];
        return;
    }
    // 计算分割轴，和分割阈值
    IndexVec left, right;
    // 没找到分割轴和分割阈值
    if (!FindSplitAxisAndThresh(points, node->axis_index_, node->split_thresh_, left, right)) {
        leaf_node_size_++;
        node->point_idx_ = points[0];
        return;
    }
    // 找到分割轴和分割阈值
    // 递归的插入元素
    const auto create_if_not_empty = [&node, this](KdTreeNode*& new_node, const IndexVec& index) {
        if (!index.empty()) {
            // 创建下一个节点
            new_node = new KdTreeNode();
            new_node->id_ = tree_node_id_++;
            Insert(index, new_node);
        }
    };
    create_if_not_empty(node->left_, left);
    create_if_not_empty(node->right_, right);
}

bool KdTree::FindSplitAxisAndThresh(const std::vector<int>& point_idx, int& axis, float& th, std::vector<int>& left,
                                    std::vector<int>& right) {
    // 计算三个轴上的分布情况
    Vec3f var;
    Vec3f mean;
    math::ComputeMeanAndCovDiag(point_idx, mean, var, [this](int idx) { return cloud_[idx]; });
    int max_i, max_j;
    // 返回最大的行，最大的列
    var.maxCoeff(&max_i, &max_j);
    // 分布最大的先进行分割
    axis = max_i;
    // 该轴上的平均值
    th = mean[axis];
    for (const auto& id : point_idx) {
        auto pt = cloud_[id];
        if (pt[axis] < th) {
            // 左子树
            left.emplace_back(id);
        } else {
            // 右子树
            right.emplace_back(id);
        }
    }
    // 这种情况不需要继续展开，直接将当前节点设为叶子就行
    if (point_idx.size() > 1 && (left.empty() || right.empty())) {
        return false;
    }
    return true;
}

void KdTree::Reset() {
    // 重置并将根节点的id置为0
    tree_node_id_ = 0;
    root_.reset(new KdTreeNode());
    // 先赋值，后++
    root_->id_ = tree_node_id_++;
    leaf_node_size_ = 0;
}

void KdTree::Clear() {
}

void KdTree::PrintAll() {
    for (const auto& np : nodes_record_) {
        auto node = np.second;
        if (node->left_ == nullptr && node->right_ == nullptr) {
            // 叶子节点，打印值
            LOG(INFO) << "leaf node: " << node->id_ << ", idx: " << node->point_idx_;
        } else {
            LOG(INFO) << "node: " << node->id_ << ", axis: " << node->axis_index_ << ", th: " << node->split_thresh_;
        }
    }
}

}  // namespace slam_learn::kdtree