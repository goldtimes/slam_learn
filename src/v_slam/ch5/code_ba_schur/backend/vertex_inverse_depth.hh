#include "v_slam/ch5/code_ba_schur/backend/vertex.hh"

namespace slam_learn::backend {
class VertexInvDepth : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexInvDepth() : Vertex(1) {
    }

    std::string TypeInfo() const {
        return "VertexInverseDepth";
    }
};
}  // namespace slam_learn::backend