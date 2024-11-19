#include "v_slam/ch5/code_ba_schur/backend/vertex.hh"

namespace slam_learn::backend {
class VertexMotion : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexMotion() : Vertex(9) {
    }

    std::string TypeInfo() const {
        return "VertexMotion";
    }
};
}  // namespace slam_learn::backend