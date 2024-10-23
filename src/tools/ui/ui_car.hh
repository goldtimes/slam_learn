#pragma once
#include <pangolin/gl/glvbo.h>
#include <vector>
#include "common/eigen_types.hh"

namespace slam_learn::ui {
class UICar {
   public:
    UICar(const Vec3f& color) : color_(color) {
    }

    /// 设置小车 Pose，重设显存中的点
    void SetPose(const SE3& pose);

    /// 渲染小车
    void Render();

   private:
    //车的顶点
    static std::vector<Vec3f> car_vertices_;
    //车的颜色
    Vec3f color_;
    // buffer data
    pangolin::GlBuffer vbo_;
};
}  // namespace slam_learn::ui