#include "overworld/Physics/PhysX/Joints/JointFixed.h"

#include <glm/gtc/quaternion.hpp>

#include "extensions/PxFixedJoint.h"
#include "overworld/Helper/GlmMath.h"
#include "overworld/Physics/PhysX/Actor.h"
#include "overworld/Physics/PhysX/Context.h"
#include "overworld/Physics/PhysX/SharedContext.h"

namespace owds::physx {
  JointFixed::JointFixed(owds::physx::Context& ctx, JointLocation location)
    : owds::JointFixed(location), ctx_(ctx) {}

  void JointFixed::setup()
  {
    const auto& sdk = owds::physx::Context::shared_ctx_->px_physics_;
    const auto quat_orientation0 = location_.origin_orientation_;
    const auto quat_orientation1 = location_.joint_orientation_;

    joint_ = ::physx::PxFixedJointCreate(
      *sdk,

      dynamic_cast<const owds::physx::Actor*>(&location_.parent_)->px_actor_.get(),
      ::physx::PxTransform(
        ::physx::PxVec3(
          static_cast<::physx::PxReal>(location_.origin_position_.x),
          static_cast<::physx::PxReal>(location_.origin_position_.y),
          static_cast<::physx::PxReal>(location_.origin_position_.z)),
        ::physx::PxQuat(
          static_cast<::physx::PxReal>(quat_orientation0.x),
          static_cast<::physx::PxReal>(quat_orientation0.y),
          static_cast<::physx::PxReal>(quat_orientation0.z),
          static_cast<::physx::PxReal>(quat_orientation0.w))),

      dynamic_cast<const owds::physx::Actor*>(&location_.child_)->px_actor_.get(),
      ::physx::PxTransform(
        ::physx::PxVec3(
          static_cast<::physx::PxReal>(location_.joint_position_.x),
          static_cast<::physx::PxReal>(location_.joint_position_.y),
          static_cast<::physx::PxReal>(location_.joint_position_.z)),
        ::physx::PxQuat(
          static_cast<::physx::PxReal>(quat_orientation1.x),
          static_cast<::physx::PxReal>(quat_orientation1.y),
          static_cast<::physx::PxReal>(quat_orientation1.z),
          static_cast<::physx::PxReal>(quat_orientation1.w))));
  }

  void JointFixed::remove()
  {
    ctx_.joints_.erase(this);
  }
} // namespace owds::physx