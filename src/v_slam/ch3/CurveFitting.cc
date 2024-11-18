#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <vector>
#include "v_slam/ch3/backend/edge.hh"
#include "v_slam/ch3/backend/problem.hh"
#include "v_slam/ch3/backend/vertex.hh"

using namespace slam_learn::backend;
using namespace std;

class CurveFittingVertex : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 构造函数，顶点的维度为3,abc优化的变量值
    CurveFittingVertex() : Vertex(3) {
    }
    virtual std::string TypeInfo() const {
        return "abc";
    }
};

class CurveFittingEdge : public Edge {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x, double y) : Edge(1, 1, {std::string("abc")}) {
        x_ = x;
        y_ = y;
    }
    virtual void ComputeResidual() override {
        // 回去顶点的值
        Eigen::Vector3d abc = verticies_[0]->Parameters();
        // 计算残差
        residual_(0) = std::exp(abc(0) * x_ * x_ + abc(1) * x_ + abc(2)) - y_;
    }
    virtual void ComputeJacobians() override {
        Eigen::Vector3d abc = verticies_[0]->Parameters();
        // 误差为1维，待优化的变量为3维abc, 1x3的雅可比矩阵
        double exp_y = std::exp(abc(0) * x_ * x_ + abc(1) * x_ + abc(2));
        Eigen::Matrix<double, 1, 3> jacobian_abc;
        // dr / da, dr / db, dr / dc
        jacobian_abc << x_ * x_ * exp_y, x_ * exp_y, exp_y;
        jacobians_[0] = jacobian_abc;
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "CurveFittingEdge";
    }

   public:
    // 观测值
    double x_;
    double y_;
};

int main(int argc, char** argv) {
    // 真实值
    double a = 1.0, b = 2.0, c = 1.0;
    int N = 100;
    double w_sigma = 1.0;
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0, w_sigma);
    // 这个problem其实只有一个顶点，存储了abc的初始值，然后又100条一元边来不断观测它
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    std::shared_ptr<CurveFittingVertex> vertex(new CurveFittingVertex);
    vertex->SetParameters(Eigen::Vector3d(0, 0, 0));
    problem.AddVertex(vertex);
    for (int i = 0; i < N; ++i) {
        double x = i / 100.0;
        double n = noise(generator);
        // 这里产生真实值，noise不是在自变量上增加，而是在y结果上增加
        double y = std::exp(a * x * x + b * x + c);
        y += n;

        std::shared_ptr<CurveFittingEdge> edge(new CurveFittingEdge(x, y));
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        // 添加顶点，1个顶点或者多个顶点
        edge->SetVertex(edge_vertex);
        problem.AddEdge(edge);
    }
    std::cout << "\nTest CurveFitting start..." << std::endl;
    /// 使用 LM 求解
    problem.Solve(30);  // 30

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertex->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout << "1.0,  2.0,  1.0" << std::endl;

    return 0;
}