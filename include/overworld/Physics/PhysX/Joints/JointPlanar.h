#ifndef OWDS_PHYSICS_PHYSX_JOINTS_JOINTPLANAR_H
#define OWDS_PHYSICS_PHYSX_JOINTS_JOINTPLANAR_H

#include "overworld/Physics/Base/Joints/JointPlanar.h"
#include "overworld/Physics/PhysX/API.h"

namespace owds::physx {
  class Context;
  class JointPlanar : public owds::JointPlanar
  {
  public:
    using owds::JointPlanar::JointPlanar;

    JointPlanar(
      owds::physx::Context& ctx,
      owds::JointLocation location);

    void setup() override;
    void remove() override;

  protected:
    owds::physx::Context& ctx_;
    PxPtr<::physx::PxD6Joint> d6joint_{};
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_JOINTS_JOINTPLANAR_H
