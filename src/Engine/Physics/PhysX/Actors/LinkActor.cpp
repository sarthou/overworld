#include "overworld/Engine/Physics/PhysX/Actors/LinkActor.h"

#include <array>
#include <cassert>
#include <cstddef>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>
#include <overworld/Utils/BitCast.h>

#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Physics/PhysX/Actors/Actor.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "overworld/Engine/Physics/PhysX/SharedContext.h"

namespace owds::physx {

  LinkActor::LinkActor(owds::physx::Context& ctx,
                       ::physx::PxArticulationLink* px_link,
                       size_t body_id,
                       const owds::Shape& collision_shape,
                       const std::vector<owds::Shape>& visual_shapes) : owds::physx::Actor(ctx, collision_shape, visual_shapes),
                                                                        px_link_(px_link),
                                                                        body_id_(body_id)
  {}

  LinkActor::~LinkActor() noexcept
  {}

  void LinkActor::setup(const std::array<double, 3>& position,
                        const std::array<double, 4>& orientation)
  {
    (void)position;
    (void)orientation;

    ctx_.physx_mutex_.lock();
    const auto& sdk = owds::physx::Context::createContext()->px_physics_;

    px_material_ = sdk->createMaterial(0.5f, 0.5f, 0.5f);
    px_base_ = px_link_.get();

    std::visit(([this](auto& elem) { setupPhysicsShape(elem); }), collision_shape_);
    for(const auto& px_geometry : px_geometries_)
    {
      px_shapes_.emplace_back(sdk->createShape(
        *px_geometry,
        *px_material_,
        false,
        ::physx::PxShapeFlag::eSCENE_QUERY_SHAPE | ::physx::PxShapeFlag::eSIMULATION_SHAPE));
      px_link_->attachShape(*px_shapes_.back());
    }

    ActorData_t* data = new ActorData_t();
    data->actor_id = unique_id_;
    data->body_id = body_id_;
    px_link_->userData = data;
    ctx_.physx_mutex_.unlock();
  }

  void LinkActor::setPhysicsEnabled(bool enabled)
  {
    (void)enabled;
    assert(false && "LinkActor cannot enabled the physics.");
  }

  void LinkActor::setSimulationEnabled(bool enabled)
  {
    (void)enabled;
    assert(false && "LinkActor cannot enabled simulation.");
  }

  void LinkActor::remove()
  {
    assert(false && "LinkActor can only be removed through their articulation");
  }

  void LinkActor::setMass(const float mass_kg)
  {
    ctx_.physx_mutex_.lock();
    px_link_->setMass(static_cast<::physx::PxReal>(mass_kg));
    ctx_.physx_mutex_.unlock();
  }

  void LinkActor::setPositionAndOrientation(const std::array<double, 3>& position, const std::array<double, 4>& orientation)
  {
    (void)position;
    (void)orientation;
    assert(false && "LinkActor can only be moved through their joint");
  }

  void LinkActor::setVelocity(const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity)
  {
    (void)linear_velocity;
    (void)angular_velocity;
    assert(false && "LinkActor velocity can only be set through their joint");
  }

} // namespace owds::physx