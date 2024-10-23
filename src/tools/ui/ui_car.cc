#include "tools/ui/ui_car.hh"

namespace slam_learn::ui {
std::vector<Vec3f> UICar::car_vertices_ = {
    // clang-format off
     { 0, 0, 0}, { 5, 0, 0},
     { 0, 0, 0}, { 0, 3, 0},
     { 0, 0, 0}, { 0, 0, 1},
    // clang-format on}
};


void UICar::SetPose(const SE3& pose) {
    std::vector<Vec3f> pts;
    for (auto& p : car_vertices_) {
        pts.emplace_back(p);
    }

    // 转换到世界系
    auto pose_f = pose.cast<float>();
    for (auto& pt : pts) {
        pt = pose_f * pt;
    }

    /// 上传到显存
    vbo_ = pangolin::GlBuffer(pangolin::GlBufferType::GlArrayBuffer, pts);
}

void UICar::Render() {
    if (vbo_.IsValid()) {
        glColor3f(color_[0], color_[1], color_[2]);
        pangolin::RenderVbo(vbo_, GL_LINES);
    }
}
}