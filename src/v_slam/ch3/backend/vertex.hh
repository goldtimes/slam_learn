#pragma once

#include "common/eigen_types.hh"

namespace slam_learn::back_end {
/**
 * 模仿g2o的vertex的定义
 */
class Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit Vertex(int num_dimension, int local_dimension = -1);

    virtual ~Vertex();

    /// 返回变量维度
    int Dimension() const;

    /// 返回变量本地维度
    int LocalDimension() const;
    /// 固定该点的估计值
    void SetFixed(bool fixed = true) {
        fixed_ = fixed;
    }

    /// 测试该点是否被固定
    bool IsFixed() const {
        return fixed_;
    }

    /// 该顶点的id
    unsigned long Id() const {
        return id_;
    }

    /// 返回参数值
    Eigen::VectorXd Parameters() const {
        return parameters_;
    }

    /// 返回参数值的引用
    Eigen::VectorXd &Parameters() {
        return parameters_;
    }

    /// 设置参数值
    void SetParameters(const Eigen::VectorXd &params) {
        parameters_ = params;
    }

    int OrderingId() const {
        return ordering_id_;
    }

    void SetOrderingId(unsigned long id) {
        ordering_id_ = id;
    };

    // 重写加法
    /// 加法，可重定义
    /// 默认是向量加
    virtual void Plus(const Eigen::VectorXd &delta);

    /// 返回顶点的名称，在子类中实现
    virtual std::string TypeInfo() const = 0;

   private:
    // 待优化的变量
    Eigen::VectorXd parameters_;
    // 局部参数的维度，比如四元素
    int local_dimension_;
    // 顶点的id
    unsigned long id_;
    // 计算雅克比的id
    unsigned long ordering_id_ = 0;
    // 是否固定
    bool fixed_ = false;

    static unsigned long global_vertex_id;
};
}  // namespace slam_learn::back_end
