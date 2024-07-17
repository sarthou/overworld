#ifndef OWDS_BASE_VERTEX_H
#define OWDS_BASE_VERTEX_H

#include <eigen3/Eigen/Eigen>

#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Vertex
  {
  public:
    std::array<float, 3> position_;
    std::array<float, 3> normal_;
    std::array<float, 2> uv_;
  };
} // namespace owds

#endif // OWDS_BASE_VERTEX_H
