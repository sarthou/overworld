#ifndef OWDS_PHYSICS_PHYSX_ACTOR_H
#define OWDS_PHYSICS_PHYSX_ACTOR_H

#include <array>
#include <memory>
#include <vector>

#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Physics/PhysX/API.h"

namespace owds::physx {

  class Context;

  class Actor : public owds::Actor
  {
  public:
    Actor(
      owds::physx::Context& ctx,
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes);

    virtual ~Actor() noexcept override = default;

    Actor(const Actor& other) = delete;
    Actor& operator=(const Actor& other) = delete;

    Actor(Actor&& other) noexcept = delete;
    Actor& operator=(Actor&& other) = delete;

    void setup(const std::array<float, 3>& position,
               const std::array<float, 4>& orientation) override = 0;
    void remove() override = 0;

    void setPhysicsEnabled(bool enabled) override = 0;
    void setSimulationEnabled(bool enabled) override = 0;
    void setMass(float mass_kg) override = 0;

    void setStaticFriction(float coefficient) override;
    void setDynamicFriction(float coefficient) override;
    void setRestitution(float coefficient) override;

    void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 4>& orientation) override = 0;

    [[nodiscard]] std::array<float, 16> getModelMatrix() const override;
    [[nodiscard]] std::pair<std::array<float, 3>, std::array<float, 3>> getPositionAndOrientation() const override;

    void setupPhysicsShape(const owds::ShapeBox& shape);
    void setupPhysicsShape(const owds::ShapeCapsule& shape);
    void setupPhysicsShape(const owds::ShapeCustomMesh& shape);
    void setupPhysicsShape(const owds::ShapeCylinder& shape);
    void setupPhysicsShape(const owds::ShapeDummy& shape);
    void setupPhysicsShape(const owds::ShapeSphere& shape);

  protected:
    owds::physx::Context& ctx_;
    // std::vector<owds::Shape> visual_shapes_;
    std::vector<std::unique_ptr<::physx::PxGeometry>> px_geometries_;
    std::vector<PxPtr<::physx::PxShape>> px_shapes_;

    PxPtr<::physx::PxMaterial> px_material_;

    PxPtr<::physx::PxRigidActor> px_base_;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_ACTOR_H