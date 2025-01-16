#ifndef OWDS_COMMON_VISUALACTOR_H
#define OWDS_COMMON_VISUALACTOR_H

#include <array>
#include <set>
#include <vector>

#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/Actor.h"

namespace owds {

  class VisualActor : public Actor
  {
  public:
    explicit VisualActor(const std::vector<owds::Shape>& visual_shapes);

    ~VisualActor() noexcept override {};
    void setup(const std::array<float, 3>& position,
               const std::array<float, 4>& orientation) override;

    void remove() override {}

    void setPhysicsEnabled(bool enabled) override;
    void setSimulationEnabled(bool enabled) override;
    void setMass(float mass_kg) override;
    void setStaticFriction(float coefficient) override;
    void setDynamicFriction(float coefficient) override;
    void setRestitution(float coefficient) override;

    /**
     * @param position Absolute position.
     * @param orientation Absolute rotation, in radians.
     */
    void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 4>& orientation) override;
    void setVelocity(const std::array<float, 3>& linear_velocity, const std::array<float, 3>& angular_velocity) override;

    AABB_t getAABB() override;
    AABB_t getLocalAABB() override;

    std::array<float, 16> getModelMatrix() const override;
    std::pair<std::array<float, 3>, std::array<float, 4>> getPositionAndOrientation() const override;

  private:
    std::array<float, 3> position_;
    std::array<float, 4> orientation_;
  };
} // namespace owds

#endif // OWDS_COMMON_VISUALACTOR_H
