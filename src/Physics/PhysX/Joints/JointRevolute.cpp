#include "overworld/Physics/PhysX/Joints/JointRevolute.h"

#include <glm/gtc/quaternion.hpp>

#include "extensions/PxRevoluteJoint.h"
#include "overworld/Helper/GlmMath.h"
#include "overworld/Physics/PhysX/Actor.h"
#include "overworld/Physics/PhysX/Context.h"
#include "overworld/Physics/PhysX/SharedContext.h"

namespace owds::physx {
  JointRevolute::JointRevolute(owds::physx::Context& ctx, owds::JointLocation location)
    : owds::JointRevolute(location), ctx_(ctx) {}

  void JointRevolute::setup()
  {
    const auto& sdk = owds::physx::Context::shared_ctx_->px_physics_;
    const auto quat_orientation0 = glm::quat(ToV3(location_.joint0_orientation_));
    const auto quat_orientation1 = glm::quat(ToV3(location_.joint1_orientation_));

    joint_ = ::physx::PxRevoluteJointCreate(
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
  }

  void JointRevolute::remove()
  {
    ctx_.joints_.erase(this);
  }
} // namespace owds::physx