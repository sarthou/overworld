#ifndef OWDS_COMMON_JOINTS_JOINTFLOATING_H
#define OWDS_COMMON_JOINTS_JOINTFLOATING_H

#include "overworld/Engine/Common/Urdf/Joints/Joint.h"

namespace owds {
  /**
   * WORK IN PROGRESS
   */
  class JointFloating : public owds::Joint
  {
  public:
    using owds::Joint::Joint;
    using owds::Joint::operator=;
  };
} // namespace owds

#endif // OWDS_COMMON_JOINTS_JOINTFLOATING_H
