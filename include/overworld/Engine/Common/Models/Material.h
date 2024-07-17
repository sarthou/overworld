#ifndef OWDS_COMMON_BASE_MATERIAL_H
#define OWDS_COMMON_BASE_MATERIAL_H

#include <string>

#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class Material
  {
  public:
    owds::Color color_rgba_;
    std::string texture_path_;
  };
} // namespace owds

#endif // OWDS_COMMON_BASE_MATERIAL_H
