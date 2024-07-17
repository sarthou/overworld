#ifndef OWDS_PHYSICS_PHYSX_JOINTS_JOINTFLOATING_H
#define OWDS_PHYSICS_PHYSX_JOINTS_JOINTFLOATING_H

#include "overworld/Engine/Common/Urdf/Joints/JointFloating.h"
#include "overworld/Physics/PhysX/API.h"

namespace owds::physx {
  class Context;

  class JointFloating final : public owds::JointFloating
  {
  public:
    using owds::JointFloating::JointFloating;

    JointFloating(
      owds::physx::Context& ctx,
      owds::JointLocation location);

    void setup() override;
    void remove() override;

  protected:
    owds::physx::Context& ctx_;
    PxPtr<::physx::PxD6Joint> d6joint_{};
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_JOINTS_JOINTFLOATING_H
