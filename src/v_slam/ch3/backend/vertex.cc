#include "v_slam/ch3/backend/vertex.hh"

namespace slam_learn::back_end {

unsigned long Vertex::global_vertex_id = 0;

Vertex::Vertex(int num_dimension, int local_dimension) {
    parameters_.resize(num_dimension, 1);
    local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;
    id_ = global_vertex_id++;
}
Vertex::~Vertex() {
}

/// 返回变量维度
int Vertex::Dimension() const {
    return parameters_.rows();
}

/// 返回变量本地维度
int Vertex::LocalDimension() const {
    return local_dimension_;
}

void Vertex::Plus(const Eigen::VectorXd &delta) {
    parameters_ += delta;
}

/// 返回顶点的名称，在子类中实现
std::string Vertex::TypeInfo() const {
    return "";
}
}  // namespace slam_learn::back_end