#pragma once
#include <Eigen/Core>
namespace slam_learn::backend {
class LossFunction {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual ~LossFunction() {
    }

    virtual double Compute(double error) const = 0;
};

/**
 * 平凡的Loss，不作任何处理
 * 使用nullptr作为loss function时效果相同
 *
 * TrivalLoss(e) = e^2
 */
class TrivalLoss : public LossFunction {
   public:
    virtual double Compute(double error) const override {
        return error * error;
    }
};

/**
 * Huber loss
 *
 * Huber(e) = e^2                      if e <= delta
 * huber(e) = delta*(2*e - delta)      if e > delta
 */
class HuberLoss : public LossFunction {
   public:
    explicit HuberLoss(double delta) : delta_(delta) {
    }

    virtual ~HuberLoss() {
    }

    virtual double Compute(double error) const override;

   private:
    double delta_;
};
}  // namespace slam_learn::backend