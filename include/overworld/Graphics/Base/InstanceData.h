#ifndef OWDS_GRAPHICS_BASE_INSTANCEDATA_H
#define OWDS_GRAPHICS_BASE_INSTANCEDATA_H

#include <eigen3/Eigen/Eigen>

#include "overworld/Graphics/Base/Color.h"

namespace owds {
  class InstanceData
  {
  public:
    std::array<float, 16> mvp_;
    owds::Color rgba_;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_INSTANCEDATA_H
