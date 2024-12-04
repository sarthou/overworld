#include "overworld/Engine/Physics/PhysX/Actors/DynamicActor.h"

#include <array>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>
#include <overworld/Utils/BitCast.h>

#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Physics/PhysX/Actors/DynamicActor.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "overworld/Engine/Physics/PhysX/SharedContext.h"

namespace owds::physx {

  DynamicActor::DynamicActor(owds::physx::Context& ctx,
                             const owds::Shape& collision_shape,
                             const std::vector<owds::Shape>& visual_shapes) : owds::physx::Actor(ctx, collision_shape, visual_shapes),
                                                                              is_kinematic_(false)
  {}

  DynamicActor::~DynamicActor() noexcept
  {
    remove();
  }

  void DynamicActor::setup(const std::array<float, 3>& position,
                           const std::array<float, 4>& orientation)
  {
    const auto& sdk = owds::physx::Context::createContext()->px_physics_;

    px_material_ = sdk->createMaterial(0.5f, 0.5f, 0.5f);

    std::visit(([this](auto& elem) { setupPhysicsShape(elem); }), collision_shape_);

    px_actor_ = sdk->createRigidDynamic(::physx::PxTransform(::physx::PxVec3(0, 0, 0)));
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

    px_actor_->setRigidBodyFlag(::physx::PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, true);

    setPositionAndOrientation(position, orientation);

    setPhysicsEnabled(false);

    ctx_.px_scene_->addActor(*px_actor_);
  }

  void DynamicActor::setPhysicsEnabled(bool enabled)
  {
    px_actor_->setRigidBodyFlag(::physx::PxRigidBodyFlag::eKINEMATIC, !enabled);
    is_kinematic_ = !enabled;
  }

  void DynamicActor::setSimulationEnabled(bool enabled)
  {
    px_actor_->setActorFlag(::physx::PxActorFlag::eDISABLE_SIMULATION, !enabled);
  }

  void DynamicActor::remove()
  {
    ctx_.px_scene_->removeActor(*px_actor_);
  }

  void DynamicActor::setMass(const float mass_kg)
  {
    px_actor_->setMass(static_cast<::physx::PxReal>(mass_kg));
  }

  void DynamicActor::setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 4>& orientation)
  {
    std::cout << "set pose of " << this->unique_id_ << " " << position[0] << " : " << position[1] << " : " << position[2] << std::endl;
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

    if(is_kinematic_)
      px_actor_->setKinematicTarget(px_transform);
    else
      px_actor_->setGlobalPose(px_transform);
  }

} // namespace owds::physx