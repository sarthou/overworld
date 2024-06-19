#include "overworld/Physics/Bullet3/Joints/JointPrismatic.h"

#include <glm/gtc/quaternion.hpp>

#include "overworld/Helper/BitCast.h"
#include "overworld/Physics/Bullet3/Actor.h"
#include "overworld/Physics/Bullet3/Context.h"

namespace owds::bullet3 {
  JointPrismatic::JointPrismatic(owds::bullet3::Context& ctx, JointLocation location)
    : owds::JointPrismatic(location), ctx_(ctx) {}

  JointPrismatic::~JointPrismatic() noexcept
  {
    ctx_.bt_scene_->removeConstraint(joint_.get());
  }

  void JointPrismatic::setup()
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

    // Leave X axis free while locking YZ linear axes
    joint_->setLimit(0, 1, 0); // Axes are considered floating when lower > upper
    joint_->setLimit(1, 0, 0);
    joint_->setLimit(2, 0, 0);

    // Lock all 3 angular axes
    joint_->setLimit(3, 0, 0);
    joint_->setLimit(4, 0, 0);
    joint_->setLimit(5, 0, 0);

    ctx_.bt_scene_->addConstraint(joint_.get());
  }

  void JointPrismatic::remove()
  {
    ctx_.joints_.erase(this);
  }

  void JointPrismatic::setLimits(const float lower_distance_limit, const float upper_distance_limit)
  {
    joint_->setLimit(0, lower_distance_limit, upper_distance_limit);
  }
} // namespace owds::bullet3