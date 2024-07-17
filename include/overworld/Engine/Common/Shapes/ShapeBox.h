#ifndef OWDS_SHAPES_BOX_H
#define OWDS_SHAPES_BOX_H

#include <array>
#include <memory>

#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Model;

  class ShapeBox
  {
  public:
    std::array<float, 3> half_extents_{};
    owds::Color color_rgba_{};
    std::reference_wrapper<owds::Model> box_model_;
  };
} // namespace owds

#endif // OWDS_SHAPES_BOX_H