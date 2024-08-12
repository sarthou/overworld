#ifndef OWDS_COMMON_SPHERE_H
#define OWDS_COMMON_SPHERE_H

#include <memory>

#include "glm/matrix.hpp"
#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Model;

  class ShapeSphere
  {
  public:
    float radius_{};
    owds::Color diffuse_color_;
    std::reference_wrapper<owds::Model> sphere_model_;
    glm::mat4 shape_transform_{1.};
  };
} // namespace owds

#endif // OWDS_COMMON_SPHERE_H
