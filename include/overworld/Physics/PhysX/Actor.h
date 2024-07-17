#ifndef OWDS_PHYSICS_PHYSX_ACTOR_H
#define OWDS_PHYSICS_PHYSX_ACTOR_H

#include <memory>
#include <vector>

#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Physics/PhysX/API.h"

namespace owds::physx {
  class Context;

  class Actor final : public owds::Actor
  {
    friend class JointContinuous;
    friend class JointFixed;
    friend class JointFloating;
    friend class JointPlanar;
    friend class JointPrismatic;
    friend class JointRevolute;

  public:
    Actor(
      owds::physx::Context& ctx,
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes);

    ~Actor() noexcept override;

    Actor(const Actor& other) = delete;
    Actor& operator=(const Actor& other) = delete;

    Actor(Actor&& other) noexcept = delete;
    Actor& operator=(Actor&& other) = delete;

    void setup() override;
    void remove() override;
    void setPhysicsEnabled(bool enabled) override;
    void setSimulationEnabled(bool enabled) override;
    void setMass(float mass_kg) override;
    void setStaticFriction(float coefficient) override;
    void setDynamicFriction(float coefficient) override;
    void setRestitution(float coefficient) override;
    void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 4>& orientation) override;

    [[nodiscard]] std::array<float, 16> getModelMatrix() const override;
    [[nodiscard]] std::pair<std::array<float, 3>, std::array<float, 3>> getPositionAndOrientation() const override;

    void setupPhysicsShape(const owds::ShapeBox& shape);
    void setupPhysicsShape(const owds::ShapeCapsule& shape);
    void setupPhysicsShape(const owds::ShapeCustomMesh& shape);
    void setupPhysicsShape(const owds::ShapeCylinder& shape);
    void setupPhysicsShape(const owds::ShapeDummy& shape);
    void setupPhysicsShape(const owds::ShapeSphere& shape);

    owds::physx::Context& ctx_;
    std::vector<owds::Shape> visual_shapes_;
    std::vector<std::unique_ptr<::physx::PxGeometry>> px_geometries_;
    std::vector<PxPtr<::physx::PxShape>> px_shapes_;

    PxPtr<::physx::PxRigidDynamic> px_actor_;
    PxPtr<::physx::PxMaterial> px_material_;

    bool is_kinematic_ = false;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_ACTOR_H