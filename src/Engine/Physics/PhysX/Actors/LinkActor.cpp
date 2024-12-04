#include "overworld/Engine/Physics/PhysX/Actors/LinkActor.h"

#include <array>
#include <cassert>
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
                       const owds::Shape& collision_shape,
                       const std::vector<owds::Shape>& visual_shapes) : owds::physx::Actor(ctx, collision_shape, visual_shapes),
                                                                        px_link_(px_link)
  {}

  LinkActor::~LinkActor() noexcept
  {}

  void LinkActor::setup(const std::array<float, 3>& position,
                        const std::array<float, 4>& orientation)
  {
    (void)position;
    (void)orientation;

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
  }

  void LinkActor::setPhysicsEnabled(bool enabled)
  {
    (void)enabled;
    assert(false && "LinkActor cannot enabled the physics. Use setSimulationEnabled instead.");
  }

  void LinkActor::setSimulationEnabled(bool enabled)
  {
    px_link_->setActorFlag(::physx::PxActorFlag::eDISABLE_SIMULATION, !enabled);
  }

  void LinkActor::remove()
  {
    assert(false && "LinkActor can only be removed through their articulation");
  }

  void LinkActor::setMass(const float mass_kg)
  {
    px_link_->setMass(static_cast<::physx::PxReal>(mass_kg));
  }

  void LinkActor::setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 4>& orientation)
  {
    (void)position;
    (void)orientation;
    assert(false && "LinkActor can only be moved through their joint");
  }

} // namespace owds::physx