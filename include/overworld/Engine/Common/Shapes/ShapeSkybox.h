#ifndef OWDS_COMMON_SKYBOX_H
#define OWDS_COMMON_SKYBOX_H

#include <memory>

#include "glm/matrix.hpp"
#include "overworld/Engine/Common/Models/Material.h"

namespace owds {
  class Model;

  class ShapeSkybox
  {
  public:
    owds::Material material_;
    std::reference_wrapper<owds::Model> skybox_model_;
    glm::mat4 shape_transform_{1.};
  };
} // namespace owds

#endif // OWDS_COMMON_SKYBOX_H