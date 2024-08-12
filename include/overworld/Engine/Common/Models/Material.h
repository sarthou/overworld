#ifndef OWDS_COMMON_BASE_MATERIAL_H
#define OWDS_COMMON_BASE_MATERIAL_H

#include <string>

#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Material
  {
  public:
    owds::Color diffuse_color_;
    owds::Color specular_color_;
    float shininess_ = -1.;
    std::string diffuse_texture_;
    std::string specular_texture_;
    std::string normal_texture_;
  };
} // namespace owds

#endif // OWDS_COMMON_BASE_MATERIAL_H
