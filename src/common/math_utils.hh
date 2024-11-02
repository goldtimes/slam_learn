#pragma once
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <Eigen/Core>
#include <cmath>
#include <numeric>
#include <vector>
namespace slam_learn::math {
constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad
constexpr double kRAD2DEG = 180.0 / M_PI;  // rad -> deg
// 非法定义
constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();

/**
 * 计算一个容器内数据的均值与对角形式协方差
 * @tparam C    容器类型
 * @tparam D    结果类型
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个D类型
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& datas, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = datas.size();
    assert(len > 1);
    // eval()立即被求值
    // accumulate的自定义求值方式
    auto sum = std::accumulate(datas.begin(), datas.end(), D::Zero().eval(),
                               [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); });
    mean = sum / len;
    // 协方差的计算，只要计算对角线的方差值
    // cwiseAbs2平方绝对值
    cov_diag = std::accumulate(datas.begin(), datas.end(), D::Zero().eval(),
                               [&getter, &mean](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) /
               (len - 1);
}

/**
 * 计算一个容器内数据的均值与矩阵形式协方差
 * @tparam C    容器类型
 * @tparam int 　数据维度
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个Eigen::Matrix<double, dim,1> 矢量类型
 */
template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C& datas, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov,
                       Getter&& getter) {
    size_t len = datas.size();
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    assert(len > 1);
    // eval()立即被求值
    // accumulate的自定义求值方式
    auto sum = std::accumulate(datas.begin(), datas.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                               [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); });
    mean = sum / len;
    // 协方差的计算
    cov = std::accumulate(datas.begin(), datas.end(), E::Zero().eval(),
                          [&getter, &mean](const E& sum, const auto& data) -> E {
                              D v = getter(data) - mean;
                              return sum + v * v.template transpose();
                          }) /
          (len - 1);
}

template <typename S>
bool FitLine2D(const std::vector<Eigen::Matrix<S, 2, 1>>& datas, Eigen::Matrix<S, 3, 1>& coeffs) {
    if (datas.size() < 3) {
        return false;
    }
    Eigen::MatrixXd A(datas.size(), 3);
    for (int i = 0; i < datas.size(); ++i) {
        A.row(i).head<2>() = datas[i].transpose();
        A.row(i)[2] = 1;
    }
    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    coeffs = svd.matrixV().col(2);
    return true;
}

template <typename S>
bool FitLine(const std::vector<Eigen::Matrix<S, 3, 1>>& datas, Eigen::Matrix<S, 3, 1>& line_start,
             Eigen::Matrix<S, 3, 1>& direction, double line_thresh = 0.2) {
    if (datas.size() < 2) {
        return false;
    }
    line_start = std::accumulate(datas.begin(), datas.end(), Eigen::Matrix<S, 3, 1>::Zero().eval()) / datas.size();
    Eigen::MatrixXd A(datas.size(), 3);
    for (int i = 0; i < datas.size(); ++i) {
        A.row(i) = (datas[i] - line_start).transpose();
    }

    Eigen::JacobiSVD svd(A, Eigen::ComputeFullV);
    direction = svd.matrixV().col(0);
    // 计算点到直线的距离
    for (const auto& pt : datas) {
        if (direction.template cross(pt - line_start).template squaredNorm() > line_thresh) {
            return false;
        }
    }
    return true;
}

template <typename S>
bool FitPlane(const std::vector<Eigen::Matrix<S, 3, 1>>& datas, Eigen::Matrix<S, 4, 1>& plane_coeffs,
              double plane_thresh = 1e-2) {
    if (datas.size() < 3) {
        return false;
    }
    // 方程
    Eigen::MatrixXd A(datas.size(), 4);
    for (int i = 0; i < datas.size(); ++i) {
        A.row(i).head<3>() = datas[i].transpose();
        A.row(i)[3] = 1.0;
    }

    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    plane_coeffs = svd.matrixV().col(3);

    for (int i = 0; i < datas.size(); ++i) {
        double err = plane_coeffs.template head<3>().dot(datas[i]) + plane_coeffs[3];
        if (err * err > plane_thresh) {
            return false;
        }
    }
    return true;
}

}  // namespace slam_learn::math