#pragma once
#include <algorithm>
#include <cmath>
namespace slam_learn::math {
constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad
constexpr double kRAD2DEG = 180.0 / M_PI;  // rad -> deg

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

}  // namespace slam_learn::math