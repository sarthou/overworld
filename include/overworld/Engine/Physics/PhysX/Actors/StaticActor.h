#ifndef OWDS_PHYSICS_PHYSX_STATICACTOR_H
#define OWDS_PHYSICS_PHYSX_STATICACTOR_H

#include <array>
#include <memory>
#include <vector>

#include "overworld/Engine/Physics/PhysX/API.h"
#include "overworld/Engine/Physics/PhysX/Actors/Actor.h"

namespace owds::physx {

  class StaticActor final : public owds::physx::Actor
  {
  public:
    StaticActor(
      owds::physx::Context& ctx,
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes);

    ~StaticActor() noexcept override;

    StaticActor(const StaticActor& other) = delete;
    StaticActor& operator=(const StaticActor& other) = delete;

    StaticActor(StaticActor&& other) noexcept = delete;
    StaticActor& operator=(StaticActor&& other) = delete;

    void setup(const std::array<float, 3>& position,
               const std::array<float, 4>& orientation) override;
    void remove() override;

    void setMass(float mass_kg) override;

    void setPhysicsEnabled(bool enabled) override;
    void setSimulationEnabled(bool enabled) override;

    void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 4>& orientation) override;
    void setVelocity(const std::array<float, 3>& linear_velocity, const std::array<float, 3>& angular_velocity) override;

  private:
    PxPtr<::physx::PxRigidStatic> px_actor_;
  };

} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_STATICACTOR_H