#ifndef OWDS_PHYSICS_BULLET3_ACTOR_H
#define OWDS_PHYSICS_BULLET3_ACTOR_H

#include <memory>
#include <vector>

#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Physics/Bullet3/API.h"

namespace owds::bullet3 {
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
      owds::bullet3::Context& ctx,
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
    void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation) override;

    [[nodiscard]] std::array<float, 16> getModelMatrix() const override;
    [[nodiscard]] std::pair<std::array<float, 3>, std::array<float, 3>> getPositionAndOrientation() const override;

    void setupPhysicsShape(const owds::ShapeBox& shape);
    void setupPhysicsShape(const owds::ShapeCapsule& shape);
    void setupPhysicsShape(const owds::ShapeCustomMesh& shape);
    void setupPhysicsShape(const owds::ShapeCylinder& shape);
    void setupPhysicsShape(const owds::ShapeDummy& shape);
    void setupPhysicsShape(const owds::ShapeSphere& shape);

    owds::bullet3::Context& ctx_;
    std::vector<owds::Shape> visual_shapes_;
    std::unique_ptr<btCollisionShape> bt_geometry_;
    std::unique_ptr<btRigidBody> bt_actor_;
    bool is_simulation_enabled_ = true;
    bool is_kinematic = false;
  };
} // namespace owds::bullet3

#endif // OWDS_PHYSICS_BULLET3_ACTOR_H
