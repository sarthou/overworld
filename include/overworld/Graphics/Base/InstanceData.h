#ifndef OWDS_GRAPHICS_BASE_INSTANCEDATA_H
#define OWDS_GRAPHICS_BASE_INSTANCEDATA_H

#include <array>

#include "overworld/Engine/Common/Models/Color.h"

namespace owds {
  class InstanceData
  {
  public:
    std::array<float, 16> mvp_;
    owds::Color rgba_;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_INSTANCEDATA_H
