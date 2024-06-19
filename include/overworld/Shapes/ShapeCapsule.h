#ifndef OWDS_SHAPES_CAPSULE_H
#define OWDS_SHAPES_CAPSULE_H

#include <memory>

namespace owds {
  class Model;

  class ShapeCapsule
  {
  public:
    float radius_{};
    float height_{};
    std::reference_wrapper<owds::Model> cylinder_model_;
    std::reference_wrapper<owds::Model> sphere_model_;
  };
} // namespace owds

#endif // OWDS_SHAPES_CAPSULE_H
