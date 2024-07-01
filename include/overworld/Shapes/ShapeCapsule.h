#ifndef OWDS_SHAPES_CAPSULE_H
#define OWDS_SHAPES_CAPSULE_H

#include <memory>
#include "overworld/Graphics/Base/Color.h"

namespace owds {
  class Model;

  /**
   * todo: Turns out we don't need this I think
   */
  class ShapeCapsule
  {
  public:
    float radius_{};
    float height_{};
    owds::Color color_rgba_{};
    std::reference_wrapper<owds::Model> cylinder_model_;
    std::reference_wrapper<owds::Model> sphere_model_;
  };
} // namespace owds

#endif // OWDS_SHAPES_CAPSULE_H
