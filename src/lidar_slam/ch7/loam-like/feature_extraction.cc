#include "lidar_slam/ch7/loam-like/feature_extraction.hh"
#include <algorithm>
#include <vector>
#include "common/sensors/point_type.hh"

namespace slam_learn::loam {

void FeatureExtraction::Extract(FullCloudPtr pc_in, CloudPtr pc_out_edge, CloudPtr pc_out_surf,
                                CloudPtr pc_out_ground) {
    int num_scans = 16;
    std::vector<CloudPtr> scans_in_each_line;
    for (int i = 0; i < num_scans; ++i) {
        // 对每条线分配一个点云
        scans_in_each_line.emplace_back(CloudPtr(new PointCloudXYZI));
    }
    for (const auto& pt : pc_in->points) {
        assert(pt.ring >= 0 && pt.ring < num_scans);
        PointXYZI p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        scans_in_each_line[pt.ring]->push_back(p);
    }
    // 遍历每条线处理曲率
    for (int i = 0; i < num_scans; ++i) {
        // 点太少，直接跳过
        if (scans_in_each_line[i]->size() < 131) {
            continue;
        }
        std::vector<IdAndValue> cloud_curvature;
        // 头尾留一点余量
        int total_size = scans_in_each_line[i]->size() - 10;
        for (int j = 5; j < scans_in_each_line[i]->size() - 5; ++j) {
            // 计算周围10个点的平均值
            double diffX = scans_in_each_line[i]->points[j - 5].x + scans_in_each_line[i]->points[j - 4].x +
                           scans_in_each_line[i]->points[j - 3].x + scans_in_each_line[i]->points[j - 2].x +
                           scans_in_each_line[i]->points[j - 1].x - 10 * scans_in_each_line[i]->points[j].x +
                           scans_in_each_line[i]->points[j + 1].x + scans_in_each_line[i]->points[j + 2].x +
                           scans_in_each_line[i]->points[j + 3].x + scans_in_each_line[i]->points[j + 4].x +
                           scans_in_each_line[i]->points[j + 5].x;
            double diffY = scans_in_each_line[i]->points[j - 5].y + scans_in_each_line[i]->points[j - 4].y +
                           scans_in_each_line[i]->points[j - 3].y + scans_in_each_line[i]->points[j - 2].y +
                           scans_in_each_line[i]->points[j - 1].y - 10 * scans_in_each_line[i]->points[j].y +
                           scans_in_each_line[i]->points[j + 1].y + scans_in_each_line[i]->points[j + 2].y +
                           scans_in_each_line[i]->points[j + 3].y + scans_in_each_line[i]->points[j + 4].y +
                           scans_in_each_line[i]->points[j + 5].y;
            double diffZ = scans_in_each_line[i]->points[j - 5].z + scans_in_each_line[i]->points[j - 4].z +
                           scans_in_each_line[i]->points[j - 3].z + scans_in_each_line[i]->points[j - 2].z +
                           scans_in_each_line[i]->points[j - 1].z - 10 * scans_in_each_line[i]->points[j].z +
                           scans_in_each_line[i]->points[j + 1].z + scans_in_each_line[i]->points[j + 2].z +
                           scans_in_each_line[i]->points[j + 3].z + scans_in_each_line[i]->points[j + 4].z +
                           scans_in_each_line[i]->points[j + 5].z;
            IdAndValue distance(j, diffX * diffX + diffY * diffY + diffZ * diffZ);
            cloud_curvature.push_back(distance);
        }
        // 分6段处理曲率
        for (int j = 0; j < 6; ++j) {
            // 每段的长度
            int sector_length = static_cast<int>(total_size / 6);
            int sector_start = sector_length * j;
            int sector_end = sector_length * (j + 1) - 1;
            // 最后一段
            if (j == 5) {
                sector_end = total_size - 1;
            }

            std::vector<IdAndValue> sub_cloud_curvature(cloud_curvature.begin() + sector_start,
                                                        cloud_curvature.begin() + sector_end);
            ExtractFromSector(scans_in_each_line[i], sub_cloud_curvature, pc_out_edge, pc_out_surf);
        }
    }
}

/**
 * 对单独一段区域提取角点和面点
 * @param pc_in
 * @param cloud_curvature
 * @param pc_out_edge
 * @param pc_out_surf
 */
void FeatureExtraction::ExtractFromSector(const CloudPtr& pc_in, std::vector<IdAndValue>& cloud_curvature,
                                          CloudPtr& pc_out_edge, CloudPtr& pc_out_surf) {
    // 从小到大排列曲率
    std::sort(cloud_curvature.begin(), cloud_curvature.end(),
              [](const IdAndValue& v1, const IdAndValue& v2) { return v1.value_ < v2.value_; });
    int largest_picked_num = 0;
    int point_info_count = 0;
    // 选取曲率最大的点，选过的点周围的点不应该考虑进去
    std::vector<int> picked_points;
    for (int i = cloud_curvature.size() - 1; i >= 0; --i) {
        int point_in_line_id = cloud_curvature[i].id_;
        auto iter = std::find(picked_points.begin(), picked_points.end(), point_in_line_id);
        // 未被选取
        if (iter == picked_points.end()) {
            // 最大的曲率都小于0.1，那么都是平面点
            if (cloud_curvature[i].value_ <= 0.1) {
                break;
            }
            //否则
            largest_picked_num++;
            picked_points.push_back(point_in_line_id);
            // 如果每条线已经选了20个角点
            if (largest_picked_num > 20) {
                break;
            } else {
                pc_out_edge->push_back(pc_in->points[point_in_line_id]);
                point_info_count++;
            }
            // 过滤周围的点
            for (int k = 1; k <= 5; ++k) {
                // 一次计算相邻的5个点
                double diffX = pc_in->points[point_in_line_id + k].x - pc_in->points[point_in_line_id + k - 1].x;
                double diffY = pc_in->points[point_in_line_id + k].y - pc_in->points[point_in_line_id + k - 1].y;
                double diffZ = pc_in->points[point_in_line_id + k].z - pc_in->points[point_in_line_id + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                    break;
                }
                picked_points.push_back(point_in_line_id + k);
            }
            for (int k = -1; k >= -5; k--) {
                double diffX = pc_in->points[point_in_line_id + k].x - pc_in->points[point_in_line_id + k + 1].x;
                double diffY = pc_in->points[point_in_line_id + k].y - pc_in->points[point_in_line_id + k + 1].y;
                double diffZ = pc_in->points[point_in_line_id + k].z - pc_in->points[point_in_line_id + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                    break;
                }
                picked_points.push_back(point_in_line_id + k);
            }
        }
    }
    // 选取曲率较小的点
    for (int i = 0; i <= (int)cloud_curvature.size() - 1; i++) {
        int ind = cloud_curvature[i].id_;
        if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
            pc_out_surf->push_back(pc_in->points[ind]);
        }
    }
}
}  // namespace slam_learn::loam