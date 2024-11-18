#include "v_slam/ch5/code_ba_schur/backend/vertex.hh"

namespace slam_learn::backend {
/**
 * 传入的四元素，流形上优化
 * Pose vertex
 * parameters: tx, ty, tz, qx, qy, qz, qw, 7 DoF
 * optimization is perform on manifold, so update is 6 DoF, left multiplication
 *
 * pose is represented as Twb in VIO case
 */
class VertexPose : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() : Vertex(7, 6) {
    }
    virtual void Plus(const Eigen::VectorXd& delta) override;

    std::string TypeInfo() const {
        return "VertexPose";
    }
};
}  // namespace slam_learn::backend