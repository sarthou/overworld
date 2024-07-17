#ifndef OWDS_PHYSICS_PHYSX_JOINTS_JOINTPRISMATIC_H
#define OWDS_PHYSICS_PHYSX_JOINTS_JOINTPRISMATIC_H

#include "overworld/Engine/Common/Urdf/Joints/JointPrismatic.h"
#include "overworld/Physics/PhysX/API.h"

namespace owds::physx {
  class Context;

  class JointPrismatic final : public owds::JointPrismatic
  {
  public:
    using owds::JointPrismatic::JointPrismatic;

    JointPrismatic(
      owds::physx::Context& ctx,
      owds::JointLocation location);

    void setup() override;
    void remove() override;

    void setLimits(float lower_distance_limit, float upper_distance_limit) override;

  protected:
    owds::physx::Context& ctx_;
    PxPtr<::physx::PxD6Joint> joint_;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_JOINTS_JOINTPRISMATIC_H
