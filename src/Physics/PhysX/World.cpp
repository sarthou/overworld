#include "overworld/Physics/PhysX/World.h"

#include "overworld/Engine/Common/Urdf/Joints/Joint.h"
#include "overworld/Helper/GlmMath.h"
#include "overworld/Physics/PhysX/Actor.h"
#include "overworld/Physics/PhysX/Context.h"
#include "overworld/Physics/PhysX/Joints/JointContinuous.h"
#include "overworld/Physics/PhysX/Joints/JointFixed.h"
#include "overworld/Physics/PhysX/Joints/JointFloating.h"
#include "overworld/Physics/PhysX/Joints/JointPlanar.h"
#include "overworld/Physics/PhysX/Joints/JointPrismatic.h"
#include "overworld/Physics/PhysX/Joints/JointRevolute.h"

namespace owds::physx {
  World::World(const std::filesystem::path& base_assets_path)
    : owds::World(base_assets_path),
      ctx_(std::make_unique<owds::physx::Context>(false, "0.0.0.0")) {}

  World::~World() = default;

  std::string World::getBackendName() const
  {
    return "PhysX";
  }

  std::string World::getFullyQualifiedBackendName() const
  {
    return getBackendName() +
           " " + std::to_string(PX_PHYSICS_VERSION_MAJOR) +
           "." + std::to_string(PX_PHYSICS_VERSION_MINOR) +
           "." + std::to_string(PX_PHYSICS_VERSION_BUGFIX);
  }

  owds::Actor& World::createActor(const owds::Shape& collision_shape, const std::vector<owds::Shape>& visual_shapes)
  {
    auto actor = std::make_unique<owds::physx::Actor>(*ctx_, collision_shape, visual_shapes);
    actor->visual_shapes_ = visual_shapes;
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
    auto joint = std::make_unique<owds::physx::JointRevolute>(
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

  owds::JointContinuous& World::createJointContinuous(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    auto joint = std::make_unique<owds::physx::JointContinuous>(
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

  owds::JointPrismatic& World::createJointPrismatic(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    auto joint = std::make_unique<owds::physx::JointPrismatic>(
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
    auto joint = std::make_unique<owds::physx::JointFixed>(
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

  owds::JointFloating& World::createJointFloating(
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    auto joint = std::make_unique<owds::physx::JointFloating>(
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

  owds::JointPlanar& World::createJointPlanar [[nodiscard]] (
    owds::Actor& parent,
    const std::array<float, 3>& origin_position,
    const std::array<float, 4>& origin_orientation,
    owds::Actor& child,
    const std::array<float, 3>& joint_position,
    const std::array<float, 4>& joint_orientation)
  {
    auto joint = std::make_unique<owds::physx::JointPlanar>(
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

  void World::setGravity(const std::array<float, 3>& gravity)
  {
    ctx_->px_scene_->setGravity(::physx::PxVec3(
      static_cast<::physx::PxReal>(gravity[0]),
      static_cast<::physx::PxReal>(gravity[1]),
      static_cast<::physx::PxReal>(gravity[2])));
  }

  void World::stepSimulation(const float delta)
  {
    ctx_->px_scene_->simulate(delta);
    ctx_->px_scene_->fetchResults(true);

    ctx_->actor_list_.clear();

    for(auto& [actor_ptr, actor] : ctx_->actors_)
    {
      ctx_->actor_list_.emplace_back(*actor);
    }
  }
} // namespace owds::physx