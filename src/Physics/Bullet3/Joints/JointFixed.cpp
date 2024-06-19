#include "overworld/Physics/Bullet3/Joints/JointFixed.h"

#include <glm/gtc/quaternion.hpp>

#include "overworld/Helper/BitCast.h"
#include "overworld/Physics/Bullet3/Actor.h"
#include "overworld/Physics/Bullet3/Context.h"

namespace owds::bullet3 {
  JointFixed::JointFixed(owds::bullet3::Context& ctx, JointLocation location)
    : owds::JointFixed(location), ctx_(ctx) {}

  JointFixed::~JointFixed() noexcept
  {
    ctx_.bt_scene_->removeConstraint(joint_.get());
  }

  void JointFixed::setup()
  {
    const auto quat_orientation0 = glm::quat(owds::BitCast<glm::vec3>(location_.joint0_orientation_));
    const auto quat_orientation1 = glm::quat(owds::BitCast<glm::vec3>(location_.joint1_orientation_));

    joint_ = std::make_unique<btGeneric6DofConstraint>(
      *dynamic_cast<const owds::bullet3::Actor*>(&location_.actor0_)->bt_actor_,
      *dynamic_cast<const owds::bullet3::Actor*>(&location_.actor1_)->bt_actor_,

      btTransform(
        btQuaternion(
          static_cast<btScalar>(quat_orientation0.x),
          static_cast<btScalar>(quat_orientation0.y),
          static_cast<btScalar>(quat_orientation0.z),
          static_cast<btScalar>(quat_orientation0.w)),
        btVector3(
          static_cast<btScalar>(location_.joint0_position_[0]),
          static_cast<btScalar>(location_.joint0_position_[1]),
          static_cast<btScalar>(location_.joint0_position_[2]))),

      btTransform(
        btQuaternion(
          static_cast<btScalar>(quat_orientation1.x),
          static_cast<btScalar>(quat_orientation1.y),
          static_cast<btScalar>(quat_orientation1.z),
          static_cast<btScalar>(quat_orientation1.w)),
        btVector3(
          static_cast<btScalar>(location_.joint1_position_[0]),
          static_cast<btScalar>(location_.joint1_position_[1]),
          static_cast<btScalar>(location_.joint1_position_[2]))),
      true);

    // Lock all 6 axes (linear XYZ + angular XYZ)
    for(auto i = 0; i < 6; i++)
    {
      joint_->setLimit(0, 0, 0);
    }

    ctx_.bt_scene_->addConstraint(joint_.get());
  }

  void JointFixed::remove()
  {
    ctx_.joints_.erase(this);
  }
} // namespace owds::bullet3