#ifndef OWDS_COMMON_BASE_COLOR_H
#define OWDS_COMMON_BASE_COLOR_H

#include <cstdint>

namespace owds {
  class Color
  {
  public:
    float r_ = -1.0f;
    float g_ = -1.0f;
    float b_ = -1.0f;
    float a_ = 1.0f;
  };
} // namespace owds

#endif // OWDS_COMMON_BASE_COLOR_H
