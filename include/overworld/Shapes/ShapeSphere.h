#ifndef OWDS_SHAPES_SPHERE_H
#define OWDS_SHAPES_SPHERE_H

#include <memory>

namespace owds {
  class Model;

  class ShapeSphere
  {
  public:
    float radius_{};
    std::reference_wrapper<owds::Model> sphere_model_;
  };
} // namespace owds

#endif // OWDS_SHAPES_SPHERE_H
