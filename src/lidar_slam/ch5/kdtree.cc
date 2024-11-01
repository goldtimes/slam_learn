#include "lidar_slam/ch5/kdtree.hh"
#include <execution>
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
        // LOG(INFO) << "id:" << id;
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

bool KdTree::GetClosestPoint(const PointXYZI& pt, std::vector<int>& closest_idx, int k) {
    if (k > leaf_node_size_) {
        LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << leaf_node_size_;
        return false;
    }
    k_ = k;
    // 默认是最大堆的队列
    std::priority_queue<NodeAndDistance> knn_result;
    // knn搜索
    // knn_result 最大堆
    Knn(lidar_utils::ToVec3f(pt), root_.get(), knn_result);
    // 排序并返回结果
    closest_idx.resize(knn_result.size());
    for (int i = closest_idx.size() - 1; i >= 0; --i) {
        // 倒序插入
        closest_idx[i] = knn_result.top().node_->point_idx_;
        knn_result.pop();
    }
    return true;
}

bool KdTree::GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k) {
    // 每个点都有k个最近邻
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

// 搜索逻辑
void KdTree::Knn(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const {
    // 如果是叶子节点
    if (node->IsLeaf()) {
        // 计算和叶子节点的距离,返回
        ComputeDisForLeaf(pt, node, result);
        return;
    }
    // 那么和轴的分割阈值做对比
    KdTreeNode *this_side, *that_side;
    if (pt[node->axis_index_] < node->split_thresh_) {
        this_side = node->left_;
        that_side = node->right_;
    } else {
        this_side = node->right_;
        that_side = node->left_;
    }
    // 递归
    Knn(pt, this_side, result);
    // 当递归退出的时候，需要判断找到最近邻与点到分割轴的距离大小关系
    // 如果最近邻的距离小于点到分割轴的距离,那么不需要探索另外的子树
    // 否则去另一个子树查找
    if (NeedExpand(pt, node, result)) {
        Knn(pt, that_side, result);
    }
}

bool KdTree::NeedExpand(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const {
    if (knn_result.size() < k_) {
        return true;
    }
    if (approximate_) {
        float d = pt[node->axis_index_] - node->split_thresh_;
        if ((d * d) < knn_result.top().distance2_ * alpha_) {
            return true;
        } else {
            return false;
        }
    } else {
        // 计算到轴平面的距离
        float dist = pt[node->axis_index_] - node->split_thresh_;
        if ((dist * dist) < knn_result.top().distance2_) {
            return true;
        } else {
            return false;
        }
    }
}

void KdTree::ComputeDisForLeaf(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const {
    // 计算距离
    float dis2 = (pt - cloud_[node->point_idx_]).squaredNorm();
    // result结果小于k，那么直接插入
    if (result.size() < k_) {
        result.emplace(node, dis2);
    } else {
        // 比较最大的元素
        if (dis2 < result.top().distance2_) {
            result.emplace(node, dis2);
            result.pop();
        }
    }
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
    // 删除左右子树
    for (const auto& pair : nodes_record_) {
        if (pair.second != root_.get()) {
            delete pair.second;
        }
    }
    // 删除根节点
    nodes_record_.clear();
    root_ = nullptr;
    leaf_node_size_ = 0;
    tree_node_id_ = 0;
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