#include "v_slam/ch5/code_ba_schur/backend/problem.hh"
#include "v_slam/ch5/code_ba_schur/backend/vertex.hh"

#include <glog/logging.h>
#include <sys/types.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include "v_slam/ch3/tic_toc.hh"

#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace slam_learn::backend {

using namespace std;

void Problem::LogoutVectorSize() {
    // LOG(INFO) <<
    //           "1 problem::LogoutVectorSize verticies_:" << verticies_.size() <<
    //           " edges:" << edges_.size();
}

Problem::Problem(ProblemType problemType) : problemType_(problemType) {
    LogoutVectorSize();
    verticies_marg_.clear();
}

Problem::~Problem() {
}

bool Problem::AddVertex(std::shared_ptr<Vertex> vertex) {
    if (verticies_.find(vertex->Id()) != verticies_.end()) {
        // LOG(WARNING) << "Vertex " << vertex->Id() << " has been added before";
        return false;
    } else {
        verticies_.insert(std::pair<unsigned long, std::shared_ptr<Vertex>>(vertex->Id(), vertex));
    }

    if (problemType_ == ProblemType::SLAM_PROBLEM) {
        if (IsPoseVertex(vertex)) {
            // 添加了顶点之后就需要重新resize H
            ResizePoseHessiansWhenAddingPose(vertex);
        }
    }

    return true;
}

void Problem::AddOrderingSLAM(std::shared_ptr<slam_learn::backend::Vertex> v) {
    if (IsPoseVertex(v)) {
        v->SetOrderingId(ordering_poses_);
        idx_pose_vertices_.insert(pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
        ordering_poses_ += v->LocalDimension();
    } else if (IsLandmarkVertex(v)) {
        v->SetOrderingId(ordering_landmarks_);
        ordering_landmarks_ += v->LocalDimension();
        idx_landmark_vertices_.insert(pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
    }
}

void Problem::ResizePoseHessiansWhenAddingPose(shared_ptr<Vertex> v) {
    int size = H_prior_.rows() + v->LocalDimension();
    H_prior_.conservativeResize(size, size);
    b_prior_.conservativeResize(size);

    b_prior_.tail(v->LocalDimension()).setZero();
    // 最后几列
    H_prior_.rightCols(v->LocalDimension()).setZero();
    // 最下面几行
    H_prior_.bottomRows(v->LocalDimension()).setZero();
}

bool Problem::IsPoseVertex(std::shared_ptr<slam_learn::backend::Vertex> v) {
    string type = v->TypeInfo();
    return type == string("VertexPose");
}

bool Problem::IsLandmarkVertex(std::shared_ptr<slam_learn::backend::Vertex> v) {
    string type = v->TypeInfo();
    return type == string("VertexPointXYZ") || type == string("VertexInverseDepth");
}

bool Problem::AddEdge(std::shared_ptr<Edge> edge) {
    if (edges_.find(edge->Id()) == edges_.end()) {
        edges_.insert(std::pair<ulong, std::shared_ptr<Edge>>(edge->Id(), edge));
    } else {
        // LOG(WARNING) << "Edge " << edge->Id() << " has been added before!";
        return false;
    }

    for (auto &vertex : edge->Verticies()) {
        vertexToEdge_.insert(std::pair<ulong, std::shared_ptr<Edge>>(vertex->Id(), edge));
    }
    return true;
}

vector<shared_ptr<Edge>> Problem::GetConnectedEdges(std::shared_ptr<Vertex> vertex) {
    vector<shared_ptr<Edge>> edges;
    auto range = vertexToEdge_.equal_range(vertex->Id());
    for (auto iter = range.first; iter != range.second; ++iter) {
        // 并且这个edge还需要存在，而不是已经被remove了
        if (edges_.find(iter->second->Id()) == edges_.end()) continue;

        edges.emplace_back(iter->second);
    }
    return edges;
}

bool Problem::RemoveVertex(std::shared_ptr<Vertex> vertex) {
    // check if the vertex is in map_verticies_
    if (verticies_.find(vertex->Id()) == verticies_.end()) {
        // LOG(WARNING) << "The vertex " << vertex->Id() << " is not in the problem!" << endl;
        return false;
    }

    // 这里要 remove 该顶点对应的 edge.
    vector<shared_ptr<Edge>> remove_edges = GetConnectedEdges(vertex);
    for (size_t i = 0; i < remove_edges.size(); i++) {
        RemoveEdge(remove_edges[i]);
    }

    if (IsPoseVertex(vertex))
        idx_pose_vertices_.erase(vertex->Id());
    else
        idx_landmark_vertices_.erase(vertex->Id());

    vertex->SetOrderingId(-1);  // used to debug
    verticies_.erase(vertex->Id());
    vertexToEdge_.erase(vertex->Id());

    return true;
}

bool Problem::RemoveEdge(std::shared_ptr<Edge> edge) {
    // check if the edge is in map_edges_
    if (edges_.find(edge->Id()) == edges_.end()) {
        // LOG(WARNING) << "The edge " << edge->Id() << " is not in the problem!" << endl;
        return false;
    }

    edges_.erase(edge->Id());
    return true;
}
bool Problem::Solve(int iterations) {
    if (edges_.size() == 0 || verticies_.size() == 0) {
        std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
        return false;
    }

    out_file_.open("data_mu.txt");
    n_iter_ = -1;

    TicToc t_solve;
    // 统计优化变量的维数，为构建 H 矩阵做准备
    SetOrdering();
    // 遍历edge, 构建 H = J^T * J 矩阵
    MakeHessian();
    // LM 初始化
    ComputeLambdaInitLM();
    // LM 算法迭代求解
    bool stop = false;
    int iter = 0;

    n_iter_++;
    out_file_ << n_iter_ << " " << currentLambda_ << std::endl;
    std::cout << n_iter_ << " " << currentLambda_ << std::endl;

    while (!stop && (iter < iterations)) {
        std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Lambda= " << currentLambda_ << std::endl;

        bool oneStepSuccess = false;
        int false_cnt = 0;
        while (!oneStepSuccess)  // 不断尝试 Lambda, 直到成功迭代一步
        {
            // setLambda
            // AddLambdatoHessianLM();
            // 第四步，解线性方程 H X = B
            SolveLinearSystem();
            //
            // RemoveLambdaHessianLM();

            // 优化退出条件1： delta_x_ 很小则退出
            if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10) {
                stop = true;
                break;
            }

            // 更新状态量 X = X+ delta_x
            UpdateStates();
            // 判断当前步是否可行以及 LM 的 lambda 怎么更新
            oneStepSuccess = IsGoodStepInLM();

            // 后续处理，
            if (oneStepSuccess) {
                // 在新线性化点 构建 hessian
                MakeHessian();
                // TODO:: 这个判断条件可以丢掉，条件 b_max <= 1e-12 很难达到，这里的阈值条件不应该用绝对值，而是相对值
                //                double b_max = 0.0;
                //                for (int i = 0; i < b_.size(); ++i) {
                //                    b_max = max(fabs(b_(i)), b_max);
                //                }
                //                // 优化退出条件2： 如果残差 b_max 已经很小了，那就退出
                //                stop = (b_max <= 1e-12);
                false_cnt = 0;
            } else {
                false_cnt++;
                RollbackStates();  // 误差没下降，回滚
            }
        }
        iter++;

        // 优化退出条件3： currentChi_ 跟第一次的chi2相比，下降了 1e6 倍则退出
        if (sqrt(currentChi_) <= stopThresholdLM_) stop = true;
    }
    out_file_.close();
    std::cout << "problem solve cost: " << t_solve.toc() << " ms" << std::endl;
    std::cout << "   makeHessian cost: " << t_hessian_cost_ << " ms" << std::endl;
    return true;
}

void Problem::SetOrdering() {
    // 每次重新计数
    ordering_poses_ = 0;
    ordering_generic_ = 0;
    ordering_landmarks_ = 0;
    int debug = 0;

    // Note:: verticies_ 是 map 类型的, 顺序是按照 id 号排序的
    for (auto vertex : verticies_) {
        ordering_generic_ += vertex.second->LocalDimension();  // 所有的优化变量总维数

        if (IsPoseVertex(vertex.second)) {
            debug += vertex.second->LocalDimension();
        }

        if (problemType_ ==
            ProblemType::SLAM_PROBLEM)  // 如果是 slam 问题，还要分别统计 pose 和 landmark 的维数，后面会对他们进行排序
        {
            AddOrderingSLAM(vertex.second);
        }

        if (IsPoseVertex(vertex.second)) {
            std::cout << vertex.second->Id() << " order: " << vertex.second->OrderingId() << std::endl;
        }
    }

    std::cout << "\n ordered_landmark_vertices_ size : " << idx_landmark_vertices_.size() << std::endl;
    if (problemType_ == ProblemType::SLAM_PROBLEM) {
        // 这里要把 landmark 的 ordering 加上 pose 的数量，就保持了 landmark 在后,而 pose 在前
        ulong all_pose_dimension = ordering_poses_;
        for (auto landmarkVertex : idx_landmark_vertices_) {
            landmarkVertex.second->SetOrderingId(landmarkVertex.second->OrderingId() + all_pose_dimension);
        }
    }

    //    CHECK_EQ(CheckOrdering(), true);
}

bool Problem::CheckOrdering() {
    if (problemType_ == ProblemType::SLAM_PROBLEM) {
        int current_ordering = 0;
        for (auto v : idx_pose_vertices_) {
            assert(v.second->OrderingId() == current_ordering);
            current_ordering += v.second->LocalDimension();
        }

        for (auto v : idx_landmark_vertices_) {
            assert(v.second->OrderingId() == current_ordering);
            current_ordering += v.second->LocalDimension();
        }
    }
    return true;
}

void Problem::MakeHessian() {
    TicToc t_h;
    // 直接构造大的 H 矩阵
    ulong size = ordering_generic_;
    MatXX H(MatXX::Zero(size, size));
    VecX b(VecX::Zero(size));

    // TODO:: accelate, accelate, accelate
    //#ifdef USE_OPENMP
    //#pragma omp parallel for
    //#endif

    // 遍历每个残差，并计算他们的雅克比，得到最后的 H = J^T * J
    for (auto &edge : edges_) {
        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        auto jacobians = edge.second->Jacobians();
        auto verticies = edge.second->Verticies();
        assert(jacobians.size() == verticies.size());
        for (size_t i = 0; i < verticies.size(); ++i) {
            auto v_i = verticies[i];
            if (v_i->IsFixed()) continue;  // Hessian 里不需要添加它的信息，也就是它的雅克比为 0

            auto jacobian_i = jacobians[i];
            ulong index_i = v_i->OrderingId();
            ulong dim_i = v_i->LocalDimension();

            MatXX JtW = jacobian_i.transpose() * edge.second->Information();
            for (size_t j = i; j < verticies.size(); ++j) {
                auto v_j = verticies[j];

                if (v_j->IsFixed()) continue;

                auto jacobian_j = jacobians[j];
                ulong index_j = v_j->OrderingId();
                ulong dim_j = v_j->LocalDimension();

                assert(v_j->OrderingId() != -1);
                MatXX hessian = JtW * jacobian_j;
                // 所有的信息矩阵叠加起来
                H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                if (j != i) {
                    // 对称的下三角
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                }
            }
            b.segment(index_i, dim_i).noalias() -= JtW * edge.second->Residual();
        }
    }
    Hessian_ = H;
    b_ = b;
    t_hessian_cost_ += t_h.toc();
    // update the error_prior
    if (err_prior_.rows() > 0) {
        b_prior_ -= H_prior_ * delta_x_.head(ordering_poses_);
    }
    Hessian_.topLeftCorner(ordering_poses_, ordering_poses_) += H_prior_;
    b_.head(ordering_poses_) += b_prior_;

    delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;
}

/*
 * Solve Hx = b, we can use PCG iterative method or use sparse Cholesky
 */
void Problem::SolveLinearSystem() {
    if (problemType_ == ProblemType::GENERIC_PROBLEM) {
        // 非 SLAM 问题直接求解
        // PCG solver
        MatXX H = Hessian_;
        for (ulong i = 0; i < Hessian_.cols(); ++i) {
            H(i, i) += currentLambda_;
        }
        //        delta_x_ = PCGSolver(H, b_, H.rows() * 2);
        delta_x_ = Hessian_.inverse() * b_;
    } else {
        // 稀疏问题 舒尔补
        int reserve_size = ordering_poses_;
        int marg_size = ordering_landmarks_;
        MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
        MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
        MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
        VecX bpp = b_.segment(0, reserve_size);
        VecX bmm = b_.segment(reserve_size, marg_size);

        // Hmm
        // 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
        MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
        for (auto landmarkVertex : idx_landmark_vertices_) {
            int idx = landmarkVertex.second->OrderingId() - reserve_size;
            int size = landmarkVertex.second->LocalDimension();
            Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
        }

        // TODO:: home work [cg]. 完成舒尔补 Hpp, bpp 代码
        MatXX tempH = Hpm * Hmm_inv;
        // H_pp_schur_ = Hessian_.block(?,?,?,?) - tempH * Hmp;
        // b_pp_schur_ = bpp - ? * ?;
        H_pp_schur_ = Hessian_.block(0, 0, reserve_size, reserve_size) - tempH * Hmp;
        b_pp_schur_ = bpp - tempH * bmm;

        // step2: solve Hpp * delta_x = bpp
        VecX delta_x_pp(VecX::Zero(reserve_size));
        // PCG Solver
        for (ulong i = 0; i < ordering_poses_; ++i) {
            H_pp_schur_(i, i) += currentLambda_;
        }

        int n = H_pp_schur_.rows() * 2;                       // 迭代次数
        delta_x_pp = PCGSolver(H_pp_schur_, b_pp_schur_, n);  // 哈哈，小规模问题，搞 pcg 花里胡哨
        delta_x_.head(reserve_size) = delta_x_pp;
        //        std::cout << delta_x_pp.transpose() << std::endl;

        // TODO:: home work [cg]. step3: solve landmark
        VecX delta_x_ll(marg_size);
        // delta_x_ll = ???;
        delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
        delta_x_.tail(marg_size) = delta_x_ll;
    }
}

void Problem::UpdateStates() {
    for (auto vertex : verticies_) {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);

        // 所有的参数 x 叠加一个增量  x_{k+1} = x_{k} + delta_x
        vertex.second->Plus(delta);
    }
    if (err_prior_.rows() > 0) {
        b_prior_ -= H_prior_ * delta_x_.head(ordering_poses_);  // update the error_prior
        err_prior_ = Jt_prior_inv_ * b_prior_.head(ordering_poses_ - 6);
    }
}

void Problem::RollbackStates() {
    for (auto vertex : verticies_) {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);

        // 之前的增量加了后使得损失函数增加了，我们应该不要这次迭代结果，所以把之前加上的量减去。
        vertex.second->Plus(-delta);
    }
    if (err_prior_.rows() > 0) {
        b_prior_ += H_prior_ * delta_x_.head(ordering_poses_);  // update the error_prior
        err_prior_ = Jt_prior_inv_ * b_prior_.head(ordering_poses_ - 6);
    }
}

/// LM
void Problem::ComputeLambdaInitLM() {
    ni_ = 2.;
    currentLambda_ = -1.;
    currentChi_ = 0.0;
    // TODO:: robust cost chi2
    for (auto edge : edges_) {
        currentChi_ += edge.second->Chi2();
    }
    if (err_prior_.rows() > 0) currentChi_ += err_prior_.norm();

    stopThresholdLM_ = 1e-6 * currentChi_;  // 迭代条件为 误差下降 1e-6 倍

    double maxDiagonal = 0;
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i) {
        maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
    }
    double tau = 1e-5;
    currentLambda_ = tau * maxDiagonal;
}

void Problem::AddLambdatoHessianLM() {
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i) {
        Hessian_(i, i) += currentLambda_;
    }
}

void Problem::RemoveLambdaHessianLM() {
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    // TODO:: 这里不应该减去一个，数值的反复加减容易造成数值精度出问题？而应该保存叠加lambda前的值，在这里直接赋值
    for (ulong i = 0; i < size; ++i) {
        Hessian_(i, i) -= currentLambda_;
    }
}

bool Problem::IsGoodStepInLM() {
    double scale = 0;
    scale = delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
    scale += 1e-3;  // make sure it's non-zero :)

    // recompute residuals after update state
    // 统计所有的残差
    double tempChi = 0.0;
    for (auto edge : edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->Chi2();
    }

    double rho = (currentChi_ - tempChi) / scale;
    if (rho > 0 && std::isfinite(tempChi))  // last step was good, 误差在下降
    {
        double alpha = 1. - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;
        n_iter_++;
        out_file_ << n_iter_ << " " << currentLambda_ << std::endl;
        std::cout << n_iter_ << " " << currentLambda_ << std::endl;
        return true;
    } else {
        currentLambda_ *= ni_;
        ni_ *= 2;
        n_iter_++;
        out_file_ << n_iter_ << " " << currentLambda_ << std::endl;
        std::cout << n_iter_ << " " << currentLambda_ << "  false" << std::endl;
        return false;
    }

    //    if(rho < 0.25) {
    //        currentLambda_ *= 2;
    //    } else if (rho > 0.75) {
    //        currentLambda_ *= 0.3;
    //    }
    //    if(rho > 0) // step acceptable
    //        return true;
    //    else
    //        return false;
}

// /** @brief conjugate gradient with perconditioning
//  *
//  *  the jacobi PCG method
//  *
//  */
Eigen::VectorXd Problem::PCGSolver(const MatXX &A, const VecX &b, int maxIter = -1) {
    assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
    int rows = b.rows();
    int n = maxIter < 0 ? rows : maxIter;
    VecX x(VecX::Zero(rows));
    MatXX M_inv = A.diagonal().asDiagonal().inverse();
    VecX r0(b);  // initial r = b - A*0 = b
    VecX z0 = M_inv * r0;
    VecX p(z0);
    VecX w = A * p;
    double r0z0 = r0.dot(z0);
    double alpha = r0z0 / p.dot(w);
    VecX r1 = r0 - alpha * w;
    int i = 0;
    double threshold = 1e-6 * r0.norm();
    while (r1.norm() > threshold && i < n) {
        i++;
        VecX z1 = M_inv * r1;
        double r1z1 = r1.dot(z1);
        double belta = r1z1 / r0z0;
        z0 = z1;
        r0z0 = r1z1;
        r0 = r1;
        p = belta * p + z1;
        w = A * p;
        alpha = r1z1 / p.dot(w);
        x += alpha * p;
        r1 -= alpha * w;
    }
    return x;
}

bool Problem::Marginalize(const std::shared_ptr<Vertex> frameVertex) {
    return true;
}
/**  0 1 2  删除关于1的信息矩阵
   0 |0,[1],2|             |0,1,2|         |0,2,1|
   1 |[3],[4],[5]|  ===>>> |6,7,8| ====>>> |6,8,7|
   2 |6,[7],8|             |3,4,5|         |3,5,4|

 */

void Problem::TestMarginalize() {
    // Add marg test
    int idx = 1;           // marg 中间那个变量
    int dim = 1;           // marg 变量的维度
    int reserve_size = 3;  // 总共变量的维度
    double delta1 = 0.1 * 0.1;
    double delta2 = 0.2 * 0.2;
    double delta3 = 0.3 * 0.3;

    int cols = 3;
    MatXX H_marg(MatXX::Zero(cols, cols));
    H_marg << 1. / delta1, -1. / delta1, 0, -1. / delta1, 1. / delta1 + 1. / delta2 + 1. / delta3, -1. / delta3, 0.,
        -1. / delta3, 1 / delta3;
    std::cout << "---------- TEST Marg: before marg------------" << std::endl;
    std::cout << H_marg << std::endl;

    // TODO:: home work [cg]. 将变量移动到右下角
    /// 准备工作： move the marg pose to the Hmm bottown right
    // 将 row i 移动矩阵最下面
    // 取出第二行
    Eigen::MatrixXd temp_rows = H_marg.block(idx, 0, dim, reserve_size);
    // 取出第三行
    Eigen::MatrixXd temp_botRows = H_marg.block(idx + dim, 0, reserve_size - idx - dim, reserve_size);
    // H_marg.block(?,?,?,?) = temp_botRows;
    // H_marg.block(?,?,?,?) = temp_rows;
    // 交换这两行的信息
    H_marg.block(idx, 0, reserve_size - idx - dim, reserve_size) = temp_botRows;
    H_marg.block(reserve_size - dim, 0, dim, reserve_size) = temp_rows;

    // 将 col i 移动矩阵最右边
    Eigen::MatrixXd temp_cols = H_marg.block(0, idx, reserve_size, dim);
    Eigen::MatrixXd temp_rightCols = H_marg.block(0, idx + dim, reserve_size, reserve_size - idx - dim);
    H_marg.block(0, idx, reserve_size, reserve_size - idx - dim) = temp_rightCols;
    H_marg.block(0, reserve_size - dim, reserve_size, dim) = temp_cols;

    std::cout << "---------- TEST Marg: 将变量移动到右下角------------" << std::endl;
    std::cout << H_marg << std::endl;
    // 后面就是对应公式的
    // A.inv() = A_rr - Arm*Amm_inv
    // Amm = 1/2(Amm + Amm.transpose())
    // Amm_inv = Amm^-1
    // tmpB = Amm * Amm_inv * Amm.inv()
    /// 开始 marg ： schur
    double eps = 1e-8;
    int m2 = dim;
    int n2 = reserve_size - dim;  // 剩余变量的维度
    Eigen::MatrixXd Amm = 0.5 * (H_marg.block(n2, n2, m2, m2) + H_marg.block(n2, n2, m2, m2).transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    Eigen::MatrixXd Amm_inv =
        saes.eigenvectors() *
        Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0))
            .asDiagonal() *
        saes.eigenvectors().transpose();

    // TODO:: home work [cg]. 完成舒尔补操作
    // Eigen::MatrixXd Arm = H_marg.block(?,?,?,?);
    // Eigen::MatrixXd Amr = H_marg.block(?,?,?,?);
    // Eigen::MatrixXd Arr = H_marg.block(?,?,?,?);
    Eigen::MatrixXd Arm = H_marg.block(0, n2, n2, m2);
    Eigen::MatrixXd Amr = H_marg.block(n2, 0, m2, n2);
    Eigen::MatrixXd Arr = H_marg.block(0, 0, n2, n2);

    Eigen::MatrixXd tempB = Arm * Amm_inv;
    Eigen::MatrixXd H_prior = Arr - tempB * Amr;

    std::cout << "---------- TEST Marg: after marg------------" << std::endl;
    std::cout << H_prior << std::endl;
}

}  // namespace slam_learn::backend
