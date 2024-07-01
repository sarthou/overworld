#include "overworld/Physics/PhysX/Joints/JointPlanar.h"

#include <glm/gtc/quaternion.hpp>

#include "overworld/Helper/GlmMath.h"
#include "overworld/Physics/PhysX/Actor.h"
#include "overworld/Physics/PhysX/Context.h"
#include "overworld/Physics/PhysX/SharedContext.h"

namespace owds::physx {
  JointPlanar::JointPlanar(owds::physx::Context& ctx, owds::JointLocation location)
    : owds::JointPlanar(location), ctx_(ctx) {}

  void JointPlanar::setup()
  {
    const auto& sdk = owds::physx::Context::shared_ctx_->px_physics_;
    const auto quat_orientation0 = ToQT(location_.joint0_orientation_);
    const auto quat_orientation1 = ToQT(location_.joint1_orientation_);

    d6joint_ = ::physx::PxD6JointCreate(
      *sdk,

      dynamic_cast<const owds::physx::Actor*>(&location_.actor0_)->px_actor_.get(),
      ::physx::PxTransform(
        ::physx::PxVec3(
          static_cast<::physx::PxReal>(location_.joint0_position_[0]),
          static_cast<::physx::PxReal>(location_.joint0_position_[1]),
          static_cast<::physx::PxReal>(location_.joint0_position_[2])),
        ::physx::PxQuat(
          static_cast<::physx::PxReal>(quat_orientation0.x),
          static_cast<::physx::PxReal>(quat_orientation0.y),
          static_cast<::physx::PxReal>(quat_orientation0.z),
          static_cast<::physx::PxReal>(quat_orientation0.w))),

      dynamic_cast<const owds::physx::Actor*>(&location_.actor1_)->px_actor_.get(),
      ::physx::PxTransform(
        ::physx::PxVec3(
          static_cast<::physx::PxReal>(location_.joint1_position_[0]),
          static_cast<::physx::PxReal>(location_.joint1_position_[1]),
          static_cast<::physx::PxReal>(location_.joint1_position_[2])),
        ::physx::PxQuat(
          static_cast<::physx::PxReal>(quat_orientation1.x),
          static_cast<::physx::PxReal>(quat_orientation1.y),
          static_cast<::physx::PxReal>(quat_orientation1.z),
          static_cast<::physx::PxReal>(quat_orientation1.w))));

    d6joint_->setMotion(::physx::PxD6Axis::eX, ::physx::PxD6Motion::eFREE);
    d6joint_->setMotion(::physx::PxD6Axis::eY, ::physx::PxD6Motion::eFREE);
    d6joint_->setMotion(::physx::PxD6Axis::eZ, ::physx::PxD6Motion::eFREE);
  }

  void JointPlanar::remove()
  {
    ctx_.joints_.erase(this);
  }
} // namespace owds::physx