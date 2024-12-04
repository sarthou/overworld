#include "overworld/Engine/Physics/PhysX/Actors/StaticActor.h"

#include <array>
#include <cassert>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>

#include "overworld/Engine/Physics/PhysX/Actors/StaticActor.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "overworld/Engine/Physics/PhysX/SharedContext.h"

namespace owds::physx {

  StaticActor::StaticActor(owds::physx::Context& ctx,
                           const owds::Shape& collision_shape,
                           const std::vector<owds::Shape>& visual_shapes) : owds::physx::Actor(ctx, collision_shape, visual_shapes)
  {}

  StaticActor::~StaticActor() noexcept
  {
    remove();
  }

  void StaticActor::setup(const std::array<float, 3>& position,
                          const std::array<float, 4>& orientation)
  {
    const auto& sdk = owds::physx::Context::createContext()->px_physics_;

    px_material_ = sdk->createMaterial(0.5f, 0.5f, 0.5f);

    const auto px_transform =
      ::physx::PxTransform(
        ::physx::PxVec3(
          static_cast<::physx::PxReal>(position[0]),
          static_cast<::physx::PxReal>(position[1]),
          static_cast<::physx::PxReal>(position[2])),
        ::physx::PxQuat(
          static_cast<::physx::PxReal>(orientation[0]),
          static_cast<::physx::PxReal>(orientation[1]),
          static_cast<::physx::PxReal>(orientation[2]),
          static_cast<::physx::PxReal>(orientation[3])));

    std::visit(([this](auto& elem) { setupPhysicsShape(elem); }), collision_shape_);

    px_actor_ = sdk->createRigidStatic(px_transform);
    px_base_ = px_actor_.get();

    for(const auto& px_geometry : px_geometries_)
    {
      px_shapes_.emplace_back(sdk->createShape(
        *px_geometry,
        *px_material_,
        false,
        ::physx::PxShapeFlag::eSCENE_QUERY_SHAPE | ::physx::PxShapeFlag::eSIMULATION_SHAPE));
      px_actor_->attachShape(*px_shapes_.back());
    }

    ctx_.px_scene_->addActor(*px_actor_);
  }

  void StaticActor::setPhysicsEnabled(bool enabled)
  {
    (void)enabled;
    assert(false && "Static actors cannot have physics.");
  }

  void StaticActor::setSimulationEnabled(bool enabled)
  {
    (void)enabled;
    assert(false && "Static actors cannot have physics.");
  }

  void StaticActor::remove()
  {
    ctx_.px_scene_->removeActor(*px_actor_);
  }

  void StaticActor::setMass(const float mass_kg)
  {
    (void)mass_kg;
  }

  void StaticActor::setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 4>& orientation)
  {
    (void)position;
    (void)orientation;
    assert(false && "Static actors cannot be moved.");
  }

} // namespace owds::physx