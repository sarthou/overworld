#ifndef OWDS_COMMON_BASE_COLOR_H
#define OWDS_COMMON_BASE_COLOR_H

#include <cstdint>

namespace owds {
  class Color
  {
  public:
    std::uint8_t r_;
    std::uint8_t g_;
    std::uint8_t b_;
    std::uint8_t a_;
  };
} // namespace owds

#endif // OWDS_COMMON_BASE_COLOR_H
