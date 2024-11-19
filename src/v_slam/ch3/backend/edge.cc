#include "v_slam/ch3/backend/edge.hh"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include "v_slam/ch3/backend/vertex.hh"

namespace slam_learn::backend {
unsigned long global_edge_id = 0;
/**
 * 构造函数，会自动化配雅可比的空间
 * @param residual_dimension 残差维度
 * @param num_verticies 顶点数量
 * @param verticies_types 顶点类型名称，可以不给，不给的话check中不会检查
 */
Edge::Edge(int residual_dimension, int num_verticies, const std::vector<std::string> &verticies_types) {
    residual_.resize(residual_dimension, 1);
    if (!verticies_types.empty()) {
        verticies_types_ = verticies_types;
    }

    jacobians_.resize(num_verticies);
    id_ = global_edge_id++;
    Eigen::MatrixXd information(residual_dimension, residual_dimension);
    information.setIdentity();
    information_ = information;
}
Edge::~Edge() {
}

/// 计算平方误差，会乘以信息矩阵
double Edge::Chi2() {
    return residual_.transpose() * information_ * residual_;
}

/// 检查边的信息是否全部设置
bool Edge::CheckValid() {
    if (!verticies_types_.empty()) {
        // check type info
        for (size_t i = 0; i < verticies_.size(); ++i) {
            if (verticies_types_[i] != verticies_[i]->TypeInfo()) {
                std::cout << "Vertex type does not match, should be " << verticies_types_[i] << ", but set to "
                          << verticies_[i]->TypeInfo() << std::endl;
                return false;
            }
        }
    }
    return true;
}
}  // namespace slam_learn::backend
