#ifndef OWDS_PHYSICS_BASE_JOINTS_JOINTPRISMATIC_H
#define OWDS_PHYSICS_BASE_JOINTS_JOINTPRISMATIC_H

#include "overworld/Physics/Base/Joints/Joint.h"

namespace owds {
  /**
   * WORK IN PROGRESS
   */
  class JointPrismatic : public owds::Joint
  {
  public:
    using owds::Joint::Joint;
    using owds::Joint::operator=;

    virtual void setLimits(float lower_distance_limit, float upper_distance_limit) = 0;
  };
} // namespace owds

#endif // OWDS_PHYSICS_BASE_JOINTS_JOINTPRISMATIC_H
