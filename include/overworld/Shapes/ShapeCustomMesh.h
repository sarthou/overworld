#ifndef OWDS_SHAPES_CUSTOMMESH_H
#define OWDS_SHAPES_CUSTOMMESH_H

#include <array>
#include <memory>

namespace owds {
  class Model;

  /**
   * WORK IN PROGRESS
   */
  class ShapeCustomMesh
  {
  public:
    std::array<float, 3> scale_ = { 1.f, 1.f, 1.f };
    std::reference_wrapper<owds::Model> custom_model_;
  };
} // namespace owds

#endif // OWDS_SHAPES_CUSTOMMESH_H
