#ifndef OWDS_PHYSICS_PHYSX_LINKACTOR_H
#define OWDS_PHYSICS_PHYSX_LINKACTOR_H

#include <array>
#include <memory>
#include <vector>

#include "overworld/Engine/Physics/PhysX/API.h"
#include "overworld/Engine/Physics/PhysX/Actors/Actor.h"

namespace owds::physx {

  class Urdf;

  class LinkActor final : public owds::physx::Actor
  {
    friend Urdf;

  public:
    LinkActor(
      owds::physx::Context& ctx,
      ::physx::PxArticulationLink* px_link,
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes);

    ~LinkActor() noexcept override;

    LinkActor(const LinkActor& other) = delete;
    LinkActor& operator=(const LinkActor& other) = delete;

    LinkActor(LinkActor&& other) noexcept = delete;
    LinkActor& operator=(LinkActor&& other) = delete;

    void setup(const std::array<double, 3>& position,
               const std::array<double, 4>& orientation) override;
    void remove() override;

    void setMass(float mass_kg) override;

    void setPhysicsEnabled(bool enabled) override;
    void setSimulationEnabled(bool enabled) override;

    void setPositionAndOrientation(const std::array<double, 3>& position, const std::array<double, 4>& orientation) override;
    void setVelocity(const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity) override;

  private:
    PxPtr<::physx::PxArticulationLink> px_link_;
  };

} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_LINKACTOR_H