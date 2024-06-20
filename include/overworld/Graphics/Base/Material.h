#ifndef OWDS_GRAPHICS_BASE_MATERIAL_H
#define OWDS_GRAPHICS_BASE_MATERIAL_H

#include <string>

#include "overworld/Graphics/Base/Color.h"

namespace owds {
  class Material
  {
  public:
    owds::Color color_rgba_;
    std::string texture_path_;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_MATERIAL_H
