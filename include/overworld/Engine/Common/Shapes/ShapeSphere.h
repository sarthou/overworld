#ifndef OWDS_COMMON_SPHERE_H
#define OWDS_COMMON_SPHERE_H

#include <memory>

#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Model;

  class ShapeSphere
  {
  public:
    float radius_{};
    owds::Color color_rgba_;
    std::reference_wrapper<owds::Model> sphere_model_;
  };
} // namespace owds

#endif // OWDS_COMMON_SPHERE_H
