#ifndef OWDS_PHYSICS_PHYSX_JOINTS_JOINTCONTINUOUS_H
#define OWDS_PHYSICS_PHYSX_JOINTS_JOINTCONTINUOUS_H

#include "overworld/Physics/Base/Joints/JointContinuous.h"
#include "overworld/Physics/PhysX/API.h"

namespace owds::physx {
  class Context;

  class JointContinuous final : public owds::JointContinuous
  {
  public:
    using owds::JointContinuous::JointContinuous;

    JointContinuous(
      owds::physx::Context& ctx,
      owds::JointLocation location);

    void setup() override;
    void remove() override;

  protected:
    owds::physx::Context& ctx_;
    PxPtr<::physx::PxD6Joint> d6joint_{};
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_JOINTS_JOINTCONTINUOUS_H
