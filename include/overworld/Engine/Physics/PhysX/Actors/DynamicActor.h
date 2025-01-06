#ifndef OWDS_PHYSICS_PHYSX_DYNAMICACTOR_H
#define OWDS_PHYSICS_PHYSX_DYNAMICACTOR_H

#include <array>
#include <memory>
#include <vector>

#include "overworld/Engine/Physics/PhysX/API.h"
#include "overworld/Engine/Physics/PhysX/Actors/Actor.h"

namespace owds::physx {

  class DynamicActor final : public owds::physx::Actor
  {
  public:
    DynamicActor(
      owds::physx::Context& ctx,
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes);

    ~DynamicActor() noexcept override;

    DynamicActor(const DynamicActor& other) = delete;
    DynamicActor& operator=(const DynamicActor& other) = delete;

    DynamicActor(DynamicActor&& other) noexcept = delete;
    DynamicActor& operator=(DynamicActor&& other) = delete;

    void setup(const std::array<double, 3>& position,
               const std::array<double, 4>& orientation) override;
    void remove() override;

    void setMass(float mass_kg) override;

    void setPhysicsEnabled(bool enabled) override;
    void setSimulationEnabled(bool enabled) override;

    void setPositionAndOrientation(const std::array<double, 3>& position, const std::array<double, 4>& orientation) override;

  private:
    PxPtr<::physx::PxRigidDynamic> px_actor_;

    bool is_kinematic_ = false;
  };

} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_DYNAMICACTOR_H