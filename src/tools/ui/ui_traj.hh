#pragma once
#include <pangolin/gl/glvbo.h>
#include <vector>
#include "common/eigen_types.hh"

namespace slam_learn::ui {
class UiTrajectory {
   public:
    UiTrajectory(const Vec3f& color) : color_(color) {
        pos_.reserve(max_size_);
    }

    /// 增加一个轨迹点
    void AddPt(const SE3& pose);

    /// 渲染此轨迹
    void Render();

    void Clear() {
        pos_.clear();
        pos_.reserve(max_size_);
        vbo_.Free();
    }

   private:
    int max_size_ = 1e6;           // 记录的最大点数
    std::vector<Vec3f> pos_;       // 轨迹记录数据
    Vec3f color_ = Vec3f::Zero();  // 轨迹颜色显示
    pangolin::GlBuffer vbo_;       // 显存顶点信息
};
}  // namespace slam_learn::ui