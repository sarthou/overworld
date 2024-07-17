#ifndef OWDS_PHYSICS_PHYSX_JOINTS_JOINTREVOLUTE_H
#define OWDS_PHYSICS_PHYSX_JOINTS_JOINTREVOLUTE_H

#include "overworld/Engine/Common/Urdf/Joints/JointRevolute.h"
#include "overworld/Physics/PhysX/API.h"

namespace owds::physx {
  class Context;

  class JointRevolute final : public owds::JointRevolute
  {
  public:
    using owds::JointRevolute::JointRevolute;

    JointRevolute(
      owds::physx::Context& ctx,
      owds::JointLocation location);

    void setup() override;
    void remove() override;

  protected:
    owds::physx::Context& ctx_;
    PxPtr<::physx::PxRevoluteJoint> joint_;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_JOINTS_JOINTREVOLUTE_H
