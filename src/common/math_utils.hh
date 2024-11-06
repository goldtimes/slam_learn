#pragma once
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <cmath>
#include <iomanip>
#include <map>
#include <numeric>
#include <vector>
#include "common/eigen_types.hh"
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

/**
 * 高斯分布合并 这里就是书上的公式了
 * @tparam S    scalar type
 * @tparam D    dimension
 * @param hist_m        历史点数
 * @param curr_n        当前点数
 * @param hist_mean     历史均值
 * @param hist_var      历史方差
 * @param curr_mean     当前均值
 * @param curr_var      当前方差
 * @param new_mean      新的均值
 * @param new_var       新的方差
 */
template <typename S, int D>
void UpdateMeanAndCov(int hist_m, int curr_n, const Eigen::Matrix<S, D, 1>& hist_mean,
                      const Eigen::Matrix<S, D, D>& hist_var, const Eigen::Matrix<S, D, 1>& curr_mean,
                      const Eigen::Matrix<S, D, D>& curr_var, Eigen::Matrix<S, D, 1>& new_mean,
                      Eigen::Matrix<S, D, D>& new_var) {
    assert(hist_m > 0);
    assert(curr_n > 0);
    new_mean = (hist_m * hist_mean + curr_n * curr_mean) / (hist_m + curr_n);
    new_var = (hist_m * (hist_var + (hist_mean - new_mean) * (hist_mean - new_mean).template transpose()) +
               curr_n * (curr_var + (curr_mean - new_mean) * (curr_mean - new_mean).template transpose())) /
              (hist_m + curr_n);
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

template <typename S>
inline SE3 Mat4ToSE3(const Eigen::Matrix<S, 4, 4>& m) {
    /// 对R做归一化，防止sophus里的检查不过
    Quatd q(m.template block<3, 3>(0, 0).template cast<double>());
    q.normalize();
    return SE3(q, m.template block<3, 1>(0, 3).template cast<double>());
}
template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<S>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

/**
 * pose 插值算法
 * @tparam T 数据类型
 * @tparam C 数据容器类型
 * @tparam FT 获取时间函数
 * @tparam FP 获取pose函数
 * @param query_time 查找时间
 * @param data  数据容器
 * @param take_pose_func 从数据中取pose的谓词，接受一个数据，返回一个SE3
 * @param result 查询结果
 * @param best_match_iter 查找到的最近匹配
 *
 * NOTE 要求query_time必须在data最大时间和最小时间之间(容许0.5s内误差)
 * data的map按时间排序
 * @return
 */
template <typename T, typename C, typename FT, typename FP>
inline bool PoseInterp(double query_time, C&& data, FT&& take_time_func, FP&& take_pose_func, SE3& result,
                       T& best_match, float time_th = 0.5) {
    if (data.empty()) {
        LOG(INFO) << "cannot interp because data is empty. ";
        return false;
    }
    // 查询的时间 > 数据中最后的时间
    double last_time = take_time_func(*data.rbegin());
    if (query_time > last_time) {
        if (query_time < (last_time + time_th)) {
            // 迭代器位置的值，也就是navistated
            best_match = *data.rbegin();
            result = take_pose_func(*data.rbegin());
            return true;
        }
        return false;
    }
    // 查询的数据小于data容器内
    // begin--query_time---end
    auto match_iter = data.begin();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;
        if (take_time_func(*iter) < query_time && take_time_func(*next_iter) >= query_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_next = match_iter;
    match_iter_next++;

    // 插值
    double dt = take_time_func(*match_iter_next) - take_time_func(*match_iter);
    double s = (query_time - take_time_func(*match_iter)) / dt;  // s=0 时为第一帧，s=1时为next

    // 出现了 dt为0的bug
    if (fabs(dt) < 1e-6) {
        best_match = *match_iter;
        result = take_pose_func(*match_iter);
        return true;
    }
    SE3 pose_first = take_pose_func(*match_iter);
    SE3 pose_next = take_pose_func(*match_iter_next);
    // se3上实现了插值算法
    result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
              pose_first.translation() * (1 - s) + pose_next.translation() * s};
    best_match = s < 0.5 ? *match_iter : *match_iter_next;
    return true;
}

/**
 * pose 插值算法
 * @tparam T    数据类型
 * @param query_time 查找时间
 * @param data  数据
 * @param take_pose_func 从数据中取pose的谓词
 * @param result 查询结果
 * @param best_match_iter 查找到的最近匹配
 *
 * NOTE 要求query_time必须在data最大时间和最小时间之间，不会外推
 * data的map按时间排序
 * @return 插值是否成功
 */
template <typename T>
bool PoseInterp(double query_time, const std::map<double, T>& data, const std::function<SE3(const T&)>& take_pose_func,
                SE3& result, T& best_match) {
    if (data.empty()) {
        LOG(INFO) << "data is empty";
        return false;
    }

    if (query_time > data.rbegin()->first) {
        LOG(INFO) << "query time is later than last, " << std::setprecision(18) << ", query: " << query_time
                  << ", end time: " << data.rbegin()->first;

        return false;
    }

    auto match_iter = data.begin();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;

        if (iter->first < query_time && next_iter->first >= query_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    match_iter_n++;
    assert(match_iter_n != data.end());

    double dt = match_iter_n->first - match_iter->first;
    double s = (query_time - match_iter->first) / dt;  // s=0 时为第一帧，s=1时为next

    SE3 pose_first = take_pose_func(match_iter->second);
    SE3 pose_next = take_pose_func(match_iter_n->second);
    result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
              pose_first.translation() * (1 - s) + pose_next.translation() * s};
    best_match = s < 0.5 ? match_iter->second : match_iter_n->second;
    return true;
}

}  // namespace slam_learn::math