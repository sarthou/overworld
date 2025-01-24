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

  void DynamicActor::setup(const std::array<double, 3>& position,
                           const std::array<double, 4>& orientation)
  {
    ctx_.physx_mutex_.lock();
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
      px_shapes_.back()->setContactOffset(0.05); // default is 0.02
      px_shapes_.back()->setRestOffset(0.0);    // default is 0.0
      px_actor_->attachShape(*px_shapes_.back());
    }

    px_actor_->setRigidBodyFlag(::physx::PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, true);
    //px_actor_->setRigidBodyFlag(::physx::PxRigidBodyFlag::eENABLE_CCD, true);
    px_actor_->setSolverIterationCounts(8, 4);

    ActorData_t* data = new ActorData_t();
    data->actor_id = unique_id_;
    data->body_id = -1;
    px_actor_->userData = data;

    ctx_.px_scene_->addActor(*px_actor_);
    ctx_.physx_mutex_.unlock();

    setPositionAndOrientation(position, orientation);

    setPhysicsEnabled(false);
  }

  void DynamicActor::setPhysicsEnabled(bool enabled)
  {
    ctx_.physx_mutex_.lock();
    px_actor_->setRigidBodyFlag(::physx::PxRigidBodyFlag::eKINEMATIC, !enabled);
    px_actor_->setActorFlag(::physx::PxActorFlag::eDISABLE_GRAVITY, !enabled);
    is_kinematic_ = !enabled;
    if(is_kinematic_ == false)
      was_kinematic_ = false;
    ctx_.physx_mutex_.unlock();
  }

  void DynamicActor::setSimulationEnabled(bool enabled)
  {
    ctx_.physx_mutex_.lock();
    px_actor_->setActorFlag(::physx::PxActorFlag::eDISABLE_GRAVITY, !enabled);
    ctx_.physx_mutex_.unlock();
  }

  void DynamicActor::remove()
  {
    ctx_.physx_mutex_.lock();
    ctx_.px_scene_->removeActor(*px_actor_);
    ctx_.physx_mutex_.unlock();
  }

  void DynamicActor::setMass(const float mass_kg)
  {
    ctx_.physx_mutex_.lock();
    px_actor_->setMass(static_cast<::physx::PxReal>(mass_kg));
    ctx_.physx_mutex_.unlock();
  }

  void DynamicActor::setPositionAndOrientation(const std::array<double, 3>& position, const std::array<double, 4>& orientation)
  {
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

    ctx_.physx_mutex_.lock();
    if(is_kinematic_ && was_kinematic_)
      px_actor_->setKinematicTarget(px_transform);
    else
    {
      px_actor_->setGlobalPose(px_transform);
      was_kinematic_ = is_kinematic_;
    }
    ctx_.physx_mutex_.unlock();
  }

  void DynamicActor::setVelocity(const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity)
  {
    if(is_kinematic_)
      return;
    
    ctx_.physx_mutex_.lock();
    px_actor_->setLinearVelocity(::physx::PxVec3(
      static_cast<::physx::PxReal>(linear_velocity[0]),
      static_cast<::physx::PxReal>(linear_velocity[1]),
      static_cast<::physx::PxReal>(linear_velocity[2])));

    px_actor_->setAngularVelocity(::physx::PxVec3(
      static_cast<::physx::PxReal>(angular_velocity[0]),
      static_cast<::physx::PxReal>(angular_velocity[1]),
      static_cast<::physx::PxReal>(angular_velocity[2])));
    ctx_.physx_mutex_.unlock();
  }

} // namespace owds::physx