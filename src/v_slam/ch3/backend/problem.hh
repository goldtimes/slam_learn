#include <fstream>
#include <map>
#include <memory>
#include <unordered_map>
#include "v_slam/ch5/code_ba_schur/backend/edge.hh"
#include "v_slam/ch5/code_ba_schur/backend/vertex.hh"

namespace slam_learn::backend {
class Problem {
   public:
    /**
     * 问题的类型
     * SLAM问题还是通用的问题
     *
     * 如果是SLAM问题那么pose和landmark是区分开的，Hessian以稀疏方式存储
     * SLAM问题只接受一些特定的Vertex和Edge
     * 如果是通用问题那么hessian是稠密的，除非用户设定某些vertex为marginalized
     */
    enum class ProblemType { SLAM_PROBLEM, GENERIC_PROBLEM };

    using ulong = unsigned long;
    // vertex的std::map key是排序的
    using HashVertex = std::map<ulong, std::shared_ptr<Vertex>>;
    using HashEdge = std::unordered_map<ulong, std::shared_ptr<Edge>>;
    using HashVertexIdToEdge = std::unordered_multimap<ulong, std::shared_ptr<Edge>>;
    using MatXX = Eigen::MatrixXd;
    using VecX = Eigen::VectorXd;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   public:
    Problem(ProblemType problemType);

    ~Problem();
    // remove 的操作涉及边缘化
    bool AddVertex(std::shared_ptr<Vertex>);
    bool RemoveVertex(std::shared_ptr<Vertex>);
    bool AddEdge(std::shared_ptr<Edge> edge);
    bool RemoveEdge(std::shared_ptr<Edge> edge);

    /**
     * 取得在优化中被判断为outlier部分的边，方便前端去除outlier
     * @param outlier_edges
     */
    void GetOutlierEdges(std::vector<std::shared_ptr<Edge>> &outlier_edges);

    /**
     * 求解此问题
     * @param iterations
     * @return
     */
    bool Solve(int iterations);

    /// 边缘化一个frame和以它为host的landmark
    bool Marginalize(std::shared_ptr<Vertex> frameVertex,
                     const std::vector<std::shared_ptr<Vertex>> &landmarkVerticies);

    bool Marginalize(const std::shared_ptr<Vertex> frameVertex);

    // test compute prior
    void TestComputePrior();
    void TestMarginalize();

   private:
    /// Solve的实现，解通用问题
    bool SolveGenericProblem(int iterations);

    /// Solve的实现，解SLAM问题
    bool SolveSLAMProblem(int iterations);

    /// 设置各顶点的ordering_index
    void SetOrdering();

    /// set ordering for new vertex in slam problem
    void AddOrderingSLAM(std::shared_ptr<Vertex> v);

    /// 构造大H矩阵
    void MakeHessian();

    /// schur求解SBA
    void SchurSBA();

    /// 解线性方程
    void SolveLinearSystem();

    /// 更新状态变量
    void UpdateStates();

    void RollbackStates();  // 有时候 update 后残差会变大，需要退回去，重来

    /// 计算并更新Prior部分
    void ComputePrior();

    /// 判断一个顶点是否为Pose顶点
    bool IsPoseVertex(std::shared_ptr<Vertex> v);

    /// 判断一个顶点是否为landmark顶点
    bool IsLandmarkVertex(std::shared_ptr<Vertex> v);

    /// 在新增顶点后，需要调整几个hessian的大小
    void ResizePoseHessiansWhenAddingPose(std::shared_ptr<Vertex> v);

    /// 检查ordering是否正确
    bool CheckOrdering();

    void LogoutVectorSize();

    /// 获取某个顶点连接到的边
    std::vector<std::shared_ptr<Edge>> GetConnectedEdges(std::shared_ptr<Vertex> vertex);

    /// Levenberg
    /// 计算LM算法的初始Lambda
    void ComputeLambdaInitLM();

    /// Hessian 对角线加上或者减去  Lambda
    void AddLambdatoHessianLM();

    void RemoveLambdaHessianLM();

    /// LM 算法中用于判断 Lambda 在上次迭代中是否可以，以及Lambda怎么缩放
    bool IsGoodStepInLM();

    /// PCG 迭代线性求解器
    VecX PCGSolver(const MatXX &A, const VecX &b, int maxIter);

   private:
    ProblemType problemType_;

    // LM算法
    double currentLambda_;
    double currentChi_;
    double stopThresholdLM_;  // LM 迭代退出阈值条件
    double ni_;               //控制 Lambda 缩放大小

    // Hessian
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;

    // 先验部分
    MatXX H_prior_;
    VecX b_prior_;
    MatXX Jt_prior_inv_;
    VecX err_prior_;
    // Heesian 的 Landmark 和 pose 部分
    MatXX H_pp_;
    VecX b_pp_;
    MatXX H_ll_;
    VecX b_ll_;
    /// SBA的Pose部分，这里有点疑问
    MatXX H_pp_schur_;
    VecX b_pp_schur_;

    // 所有顶点
    HashVertex verticies_;
    // 所有边
    HashEdge edges_;
    // id查询edge
    HashVertexIdToEdge vertexToEdge_;
    /// Ordering related
    ulong ordering_poses_ = 0;
    ulong ordering_landmarks_ = 0;
    ulong ordering_generic_ = 0;
    // 以ordering排序的pose顶点
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_pose_vertices_;
    // 以ordering排序的landmark顶点
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_landmark_vertices_;
    // verticies need to marg. <Ordering_id_, Vertex>
    HashVertex verticies_marg_;
    bool bDebug = false;
    double t_hessian_cost_ = 0.0;
    double t_PCGsovle_cost_ = 0.0;

    std::ofstream out_file_;
    int n_iter_;
};
}  // namespace slam_learn::backend
