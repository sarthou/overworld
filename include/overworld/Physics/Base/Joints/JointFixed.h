#ifndef OWDS_PHYSICS_BASE_JOINTS_JOINTFIXED_H
#define OWDS_PHYSICS_BASE_JOINTS_JOINTFIXED_H

#include "overworld/Physics/Base/Joints/Joint.h"

namespace owds {
  /**
   * WORK IN PROGRESS
   */
  class JointFixed : public owds::Joint
  {
  public:
    using owds::Joint::Joint;
    using owds::Joint::operator=;
  };
} // namespace owds

#endif // OWDS_PHYSICS_BASE_JOINTS_JOINTFIXED_H
