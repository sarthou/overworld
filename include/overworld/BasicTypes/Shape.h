#ifndef OWDS_SHAPE_H
#define OWDS_SHAPE_H

#include <array>
#include <string>

namespace owds {

  enum ShapeType_e
  {
    SHAPE_NONE,
    SHAPE_CUBE,
    SHAPE_SPEHERE,
    SHAPE_CYLINDER,
    SHAPE_MESH = 10
  };

  struct Shape_t
  {
    ShapeType_e type;
    std::string visual_mesh_resource;
    std::string colision_mesh_resource;
    std::array<double, 4> color;
    std::array<double, 3> scale;
    std::string texture;
    std::string normal_map;
    std::string specular_texture;

    Shape_t() : type(SHAPE_NONE)
    {
      color.fill(0.);
      scale = {1., 1., 1.};
    }
  };

} // namespace owds

#endif // OWDS_SHAPE_H