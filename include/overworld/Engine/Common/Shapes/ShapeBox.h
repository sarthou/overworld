#ifndef OWDS_COMMON_BOX_H
#define OWDS_COMMON_BOX_H

#include <array>
#include <memory>

#include "glm/matrix.hpp"
#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Model;

  class ShapeBox
  {
  public:
    std::array<float, 3> half_extents_{};
    owds::Color diffuse_color_{};
    std::reference_wrapper<owds::Model> box_model_;
    glm::mat4 shape_transform_{1.};
  };
} // namespace owds

#endif // OWDS_COMMON_BOX_H