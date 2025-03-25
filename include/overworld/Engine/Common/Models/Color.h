#ifndef OWDS_COMMON_BASE_COLOR_H
#define OWDS_COMMON_BASE_COLOR_H

#include <cstdint>
#include <array>

namespace owds {
  class Color
  {
  public:
    float r_;
    float g_;
    float b_;
    float a_;

    Color() : r_(-1.),
              g_(-1.),
              b_(-1.),
              a_(0.)
    {}

    Color(const std::array<float, 3>& rgb) : r_(rgb.at(0)),
                                             g_(rgb.at(1)),
                                             b_(rgb.at(2)),
                                             a_(0.)
    {}

    Color(const std::array<float, 4>& rgba) : r_(rgba.at(0)),
                                              g_(rgba.at(1)),
                                              b_(rgba.at(2)),
                                              a_(rgba.at(3))
    {}
  };
} // namespace owds

#endif // OWDS_COMMON_BASE_COLOR_H
