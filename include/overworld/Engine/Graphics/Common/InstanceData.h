#ifndef OWDS_GRAPHICS_COMMON_INSTANCEDATA_H
#define OWDS_GRAPHICS_COMMON_INSTANCEDATA_H

#include <glm/matrix.hpp>

#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class InstanceData
  {
  public:
    glm::mat4 mvp_;
    owds::Color rgba_;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_COMMON_INSTANCEDATA_H
