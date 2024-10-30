#include "lidar_slam/ch5/bfnn.hh"

#include <execution>

namespace slam_learn::bfnn {
// 遍历所有的点，找到离point最近的点
int bfnn_point(CloudPtr cloud, const Vec3f& search_point) {
    auto it = std::min_element(cloud->points.begin(), cloud->points.end(),
                               [&search_point](const PointXYZI& pt1, const PointXYZI& pt2) -> bool {
                                   return (pt1.getVector3fMap() - search_point).squaredNorm() <
                                          (pt2.getVector3fMap() - search_point).squaredNorm();
                               });
    return it - cloud->begin();
}
std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f& point, int k) {
    struct IndexAndDist {
        IndexAndDist() {
        }
        IndexAndDist(int index, double dist2) : index_(index), dist2_(dist2) {
        }
        int index_ = 0;
        double dist2_ = 0.0;
    };

    std::vector<IndexAndDist> index_and_dist(cloud->size());
    for (int i = 0; i < cloud->size(); ++i) {
        double dist2 = (cloud->points[i].getVector3fMap() - point).squaredNorm();
        index_and_dist[i] = IndexAndDist(i, dist2);
    }
    // sort
    std::sort(index_and_dist.begin(), index_and_dist.end(),
              [](const auto& a1, const auto& a2) { return a1.dist2_ < a2.dist2_; });
    // 获取index
    std::vector<int> ret;
    std::transform(index_and_dist.begin(), index_and_dist.begin() + k, std::back_inserter(ret),
                   [](const auto& d1) { return d1.index_; });
    return ret;
}
}  // namespace slam_learn::bfnn