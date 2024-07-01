#ifndef OWDS_PHYSICS_BASE_JOINTS_JOINT_H
#define OWDS_PHYSICS_BASE_JOINTS_JOINT_H

#include "overworld/Physics/Base/JointLocation.h"

namespace owds {
  class Joint
  {
  protected:
    explicit Joint(const owds::JointLocation& location);

  public:
    Joint(const Joint& other) = delete;
    Joint& operator=(const Joint& other) = delete;

    Joint(Joint&& other) noexcept = delete;
    Joint& operator=(Joint&& other) = delete;

    virtual ~Joint() noexcept;
    virtual void setup() = 0;
    virtual void remove() = 0;

  protected:
    owds::JointLocation location_;
  };
} // namespace owds

#endif // OWDS_PHYSICS_BASE_JOINTS_JOINT_H
