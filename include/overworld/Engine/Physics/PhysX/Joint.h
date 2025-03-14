#ifndef OWDS_PHYSICS_PHYSX_JOINT_H
#define OWDS_PHYSICS_PHYSX_JOINT_H

#include "overworld/Engine/Common/Urdf/Joint.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"
#include "overworld/Engine/Physics/PhysX/API.h"

namespace owds::physx {

  class Context;

  class Joint : public owds::Joint
  {
  public:
    Joint(owds::physx::Context& ctx,
          ::physx::PxArticulationJointReducedCoordinate* px_joint,
          urdf::JointType_e type, int axis, int direction = 1);

    ~Joint() noexcept override = default;

    Joint(const Joint& other) = delete;
    Joint& operator=(const Joint& other) = delete;

    Joint(Joint&& other) noexcept = delete;
    Joint& operator=(Joint&& other) = delete;

    void setPosition(double position) override;
    void setVelocity(double velocity) override;

  protected:
    owds::physx::Context& ctx_;

    ::physx::PxArticulationAxis::Enum px_axis_;

    PxPtr<::physx::PxArticulationJointReducedCoordinate> px_base_;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_JOINT_H