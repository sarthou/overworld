#include "overworld/Physics/Bullet3/World.h"

#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/JointLocation.h"
#include "overworld/Physics/Bullet3/Actor.h"
#include "overworld/Physics/Bullet3/Context.h"
#include "overworld/Physics/Bullet3/Joints/JointPrismatic.h"
#include "overworld/Utils/GlmMath.h"

namespace owds::bullet3 {
  World::World(const std::filesystem::path& base_assets_path)
    : owds::World(base_assets_path),
      ctx_(std::make_unique<owds::bullet3::Context>()) {}

  World::~World() = default;

  std::string World::getBackendName() const
  {
    return "Bullet";
  }

  std::string World::getFullyQualifiedBackendName() const
  {
    const auto version_string = std::to_string(btGetVersion());
    return getBackendName() +
           " " + version_string.front() +
           "." + version_string.substr(1);
  }

  owds::Actor& World::createActor(const owds::Shape& collision_shape, const std::vector<owds::Shape>& visual_shapes)
  {
    auto actor = std::make_unique<owds::bullet3::Actor>(*ctx_, collision_shape, visual_shapes);
    actor->setup();
    const auto actor_ptr = actor.get();
    return *(ctx_->actors_[actor_ptr] = std::move(actor), actor_ptr);
  }

  const std::vector<std::reference_wrapper<owds::Actor>>& World::getActors() const
  {
    return ctx_->actor_list_;
  }

  owds::JointRevolute& World::createJointRevolute(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    (void)parent;
    (void)origin_position;
    (void)origin_orientation;
    (void)child;
    (void)joint_position;
    (void)joint_orientation;
    assert(false && "not implemented");
  }

  owds::JointContinuous& World::createJointContinuous(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    (void)parent;
    (void)origin_position;
    (void)origin_orientation;
    (void)child;
    (void)joint_position;
    (void)joint_orientation;
    assert(false && "not implemented");
  }

  owds::JointPrismatic& World::createJointPrismatic(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    auto joint = std::make_unique<owds::bullet3::JointPrismatic>(
      *ctx_,
      owds::JointLocation(
        parent,
        ToV3(origin_position),
        ToQT(origin_orientation),
        child,
        ToV3(joint_position),
        ToQT(joint_orientation)));

    joint->setup();

    const auto joint_ptr = joint.get();
    return *(ctx_->joints_[joint_ptr] = std::move(joint), joint_ptr);
  }

  owds::JointFixed& World::createJointFixed(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    (void)parent;
    (void)origin_position;
    (void)origin_orientation;
    (void)child;
    (void)joint_position;
    (void)joint_orientation;
    assert(false && "not implemented");
  }

  owds::JointFloating& World::createJointFloating(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    (void)parent;
    (void)origin_position;
    (void)origin_orientation;
    (void)child;
    (void)joint_position;
    (void)joint_orientation;
    assert(false && "not implemented");
  }

  owds::JointPlanar& World::createJointPlanar(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    (void)parent;
    (void)origin_position;
    (void)origin_orientation;
    (void)child;
    (void)joint_position;
    (void)joint_orientation;
    assert(false && "not implemented");
  }

  void World::setGravity(const std::array<float, 3>& gravity)
  {
    ctx_->bt_scene_->setGravity(btVector3(
      static_cast<btScalar>(gravity[0]),
      static_cast<btScalar>(gravity[1]),
      static_cast<btScalar>(gravity[2])));
  }

  void World::stepSimulation(const float delta)
  {
    ctx_->bt_scene_->stepSimulation(delta, 10);

    ctx_->actor_list_.clear();

    for(auto& [actor_ptr, actor] : ctx_->actors_)
    {
      ctx_->actor_list_.emplace_back(*actor);
    }
  }
} // namespace owds::bullet3