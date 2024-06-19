#ifndef OWDS_PHYSICS_BASE_JOINTS_JOINTLOCATION_H
#define OWDS_PHYSICS_BASE_JOINTS_JOINTLOCATION_H

#include <array>

namespace owds {
  class Actor;
  class JointLocation
  {
  public:
    JointLocation(
      owds::Actor& actor0,
      const std::array<float, 3>& joint0_position,
      const std::array<float, 3>& joint0_orientation,
      owds::Actor& actor1,
      const std::array<float, 3>& joint1_position,
      const std::array<float, 3>& joint1_orientation)
      : actor0_(actor0),
        joint0_position_(joint0_position),
        joint0_orientation_(joint0_orientation),
        actor1_(actor1),
        joint1_position_(joint1_position),
        joint1_orientation_(joint1_orientation)
    {}

    owds::Actor& actor0_;
    std::array<float, 3> joint0_position_;
    std::array<float, 3> joint0_orientation_;
    owds::Actor& actor1_;
    std::array<float, 3> joint1_position_;
    std::array<float, 3> joint1_orientation_;
  };
} // namespace owds

#endif // OWDS_PHYSICS_BASE_JOINTS_JOINTLOCATION_H
