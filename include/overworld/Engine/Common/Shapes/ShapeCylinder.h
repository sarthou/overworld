#ifndef OWDS_COMMON_CYLINDER_H
#define OWDS_COMMON_CYLINDER_H

#include <memory>

#include "glm/matrix.hpp"
#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Model;

  class ShapeCylinder
  {
  public:
    float radius_{};
    float height_{};
    owds::Color diffuse_color_;
    std::reference_wrapper<owds::Model> cylinder_model_;
    glm::mat4 shape_transform_{1.};
  };
} // namespace owds

#endif // OWDS_COMMON_CYLINDER_H