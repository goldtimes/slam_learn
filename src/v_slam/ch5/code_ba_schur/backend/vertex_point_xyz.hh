#include "v_slam/ch5/code_ba_schur/backend/vertex.hh"

namespace slam_learn::backend {
class VertexPointXYZ : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPointXYZ() : Vertex(3) {
    }

    std::string TypeInfo() const {
        return "VertexPointXYZ";
    }
};
}  // namespace slam_learn::backend