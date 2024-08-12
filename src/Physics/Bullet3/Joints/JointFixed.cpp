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
    const auto quat_orientation0 = location_.origin_orientation_;
    const auto quat_orientation1 = location_.joint_orientation_;

    joint_ = std::make_unique<btGeneric6DofConstraint>(
      *dynamic_cast<const owds::bullet3::Actor*>(&location_.parent_)->bt_actor_,
      *dynamic_cast<const owds::bullet3::Actor*>(&location_.child_)->bt_actor_,

      btTransform(
        btQuaternion(
          static_cast<btScalar>(quat_orientation0.x),
          static_cast<btScalar>(quat_orientation0.y),
          static_cast<btScalar>(quat_orientation0.z),
          static_cast<btScalar>(quat_orientation0.w)),
        btVector3(
          static_cast<btScalar>(location_.origin_position_.x),
          static_cast<btScalar>(location_.origin_position_.y),
          static_cast<btScalar>(location_.origin_position_.z))),

      btTransform(
        btQuaternion(
          static_cast<btScalar>(quat_orientation1.x),
          static_cast<btScalar>(quat_orientation1.y),
          static_cast<btScalar>(quat_orientation1.z),
          static_cast<btScalar>(quat_orientation1.w)),
        btVector3(
          static_cast<btScalar>(location_.joint_position_.x),
          static_cast<btScalar>(location_.joint_position_.y),
          static_cast<btScalar>(location_.joint_position_.z))),
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