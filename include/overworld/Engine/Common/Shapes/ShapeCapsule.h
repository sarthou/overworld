#ifndef OWDS_COMMON_CAPSULE_H
#define OWDS_COMMON_CAPSULE_H

#include <memory>

#include "glm/matrix.hpp"
#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Model;

  /**
   * todo: Turns out we don't need this I think
   */
  class ShapeCapsule
  {
  public:
    float radius_ = 0.;
    float height_ = 0.;
    owds::Color diffuse_color_;
    std::reference_wrapper<owds::Model> cylinder_model_;
    std::reference_wrapper<owds::Model> sphere_model_;
    glm::mat4 shape_transform_{1.};
  };
} // namespace owds

#endif // OWDS_COMMON_CAPSULE_H
