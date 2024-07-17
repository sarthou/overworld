#ifndef OWDS_COMMON_JOINTS_JOINTFIXED_H
#define OWDS_COMMON_JOINTS_JOINTFIXED_H

#include "overworld/Engine/Common/Urdf/Joints/Joint.h"

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

#endif // OWDS_COMMON_JOINTS_JOINTFIXED_H
