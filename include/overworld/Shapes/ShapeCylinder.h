#ifndef OWDS_SHAPES_CYLINDER_H
#define OWDS_SHAPES_CYLINDER_H

#include <memory>

namespace owds {
  class Model;

  class ShapeCylinder
  {
  public:
    float radius_{};
    float height_{};
    std::reference_wrapper<owds::Model>  cylinder_model_;
  };
} // namespace owds

#endif // OWDS_SHAPES_CYLINDER_H