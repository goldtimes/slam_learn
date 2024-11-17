#pragma once

#include <memory>
#include <string>
#include <vector>
#include "common/eigen_types.hh"

namespace slam_learn::backend {
class Vertex;

class Edge {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   private:
    unsigned long id_;  // 边的id
    int ordering_id_;   // edge id in problem
    std::vector<std::string> verticies_types_;
    // 存储几个顶点
    std::vector<std::shared_ptr<Vertex>> verticies_;
    // 残差
    Eigen::VectorXd residual_;
    // 对顶点的jacobian矩阵
    std::vector<Eigen::MatrixXd> jacobians_;
    // 信息矩阵
    Eigen::MatrixXd information_;
    // 观测的数据
    Eigen::VectorXd observation_;
};
}  // namespace slam_learn::backend