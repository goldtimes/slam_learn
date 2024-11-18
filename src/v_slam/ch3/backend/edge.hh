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

    /**
     * 构造函数，会自动化配雅可比的空间
     * @param residual_dimension 残差维度
     * @param num_verticies 顶点数量
     * @param verticies_types 顶点类型名称，可以不给，不给的话check中不会检查
     */
    explicit Edge(int residual_dimension, int num_verticies,
                  const std::vector<std::string> &verticies_types = std::vector<std::string>());

    virtual ~Edge();
    int OrderingId() const {
        return ordering_id_;
    }

    void SetOrderingId(int id) {
        ordering_id_ = id;
    };

    /// 设置信息矩阵, information_ = sqrt_Omega = w
    void SetInformation(const Eigen::MatrixXd &information) {
        information_ = information;
    }

    /// 返回残差
    Eigen::VectorXd Residual() const {
        return residual_;
    }

    /// 返回雅可比
    std::vector<Eigen::MatrixXd> Jacobians() const {
        return jacobians_;
    }

    /// 返回信息矩阵
    Eigen::MatrixXd Information() const {
        return information_;
    }

    /// 设置观测信息
    void SetObservation(const Eigen::VectorXd &observation) {
        observation_ = observation;
    }

    /// 返回观测信息
    Eigen::VectorXd Observation() const {
        return observation_;
    }

    /// 返回id
    unsigned long Id() const {
        return id_;
    }

    /**
     * 设置一个顶点
     * @param vertex 对应的vertex对象
     */
    bool AddVertex(std::shared_ptr<Vertex> vertex) {
        verticies_.emplace_back(vertex);
        return true;
    }

    /**
     * 设置一些顶点
     * @param vertices 顶点，按引用顺序排列
     * @return
     */
    bool SetVertex(const std::vector<std::shared_ptr<Vertex>> &vertices) {
        verticies_ = vertices;
        return true;
    }

    /// 返回第i个顶点
    std::shared_ptr<Vertex> GetVertex(int i) {
        return verticies_[i];
    }

    /// 返回所有顶点
    std::vector<std::shared_ptr<Vertex>> Verticies() const {
        return verticies_;
    }

    /// 返回关联顶点个数
    size_t NumVertices() const {
        return verticies_.size();
    }

    /// 返回边的类型信息，在子类中实现
    virtual std::string TypeInfo() const = 0;

    /// 计算残差，由子类实现
    virtual void ComputeResidual() = 0;

    /// 计算雅可比，由子类实现
    /// 本后端不支持自动求导，需要实现每个子类的雅可比计算方法
    virtual void ComputeJacobians() = 0;

    //    ///计算该edge对Hession矩阵的影响，由子类实现
    //    virtual void ComputeHessionFactor() = 0;

    /// 计算平方误差，会乘以信息矩阵
    double Chi2();
    bool CheckValid();

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