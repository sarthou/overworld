#ifndef OWDS_COMMON_JOINTLOCATION_H
#define OWDS_COMMON_JOINTLOCATION_H

#include <array>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

namespace owds {
  class Actor;
  class JointLocation
  {
  public:
    JointLocation(
      owds::Actor& parent,
      const glm::vec3& origin_position,
      const glm::quat& origin_orientation,
      owds::Actor& child,
      const glm::vec3& joint_position,
      const glm::quat& joint_orientation)
      : parent_(parent),
        origin_position_(origin_position),
        origin_orientation_(origin_orientation),
        child_(child),
        joint_position_(joint_position),
        joint_orientation_(joint_orientation)
    {}

    owds::Actor& parent_;
    glm::vec3 origin_position_;
    glm::quat origin_orientation_;
    owds::Actor& child_;
    glm::vec3 joint_position_;
    glm::quat joint_orientation_;
  };
} // namespace owds

#endif // OWDS_COMMON_JOINTLOCATION_H
