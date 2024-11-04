#pragma once

#include "common/sensors/point_type.hh"

namespace slam_learn::loam {

class FeatureExtraction {
   public:
    // 线Id和曲率的结构体
    struct IdAndValue {
        IdAndValue() {
        }
        IdAndValue(int id, double value) : id_(id), value_(value) {
        }
        int id_;
        // 曲率
        double value_;
    };

   public:
    FeatureExtraction() {
    }

    void Extract(FullCloudPtr pc_in, CloudPtr pc_out_edge, CloudPtr pc_out_surf, CloudPtr pc_out_ground = nullptr);

    /**
     * 对单独一段区域提取角点和面点
     * @param pc_in
     * @param cloud_curvature
     * @param pc_out_edge
     * @param pc_out_surf
     */
    void ExtractFromSector(const CloudPtr& pc_in, std::vector<IdAndValue>& cloud_curvature, CloudPtr& pc_out_edge,
                           CloudPtr& pc_out_surf);

   private:
};

}  // namespace slam_learn::loam