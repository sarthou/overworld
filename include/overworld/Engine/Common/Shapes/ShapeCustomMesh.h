#ifndef OWDS_COMMON_CUSTOMMESH_H
#define OWDS_COMMON_CUSTOMMESH_H

#include <array>
#include <glm/vec3.hpp>
#include <memory>

#include "overworld/Engine/Common/Models/Material.h"

namespace owds {
  class Model;

  /**
   * WORK IN PROGRESS
   */
  class ShapeCustomMesh
  {
  public:
    glm::vec3 scale_ = {1.f, 1.f, 1.f};
    owds::Material material_;
    std::reference_wrapper<owds::Model> custom_model_;
  };
} // namespace owds

#endif // OWDS_COMMON_CUSTOMMESH_H
