#ifndef OWDS_COMMON_ACTOR_H
#define OWDS_COMMON_ACTOR_H

#include <array>
#include <set>
#include <vector>

#include "overworld/Engine/Common/Shapes/Shape.h"

namespace owds {
  class Joint;

  struct ActorData_t
  {
    size_t actor_id;
    size_t body_id;
  };

  class Actor
  {
    friend class Joint;

  protected:
    explicit Actor(
      const owds::Shape& collision_shape,
      const std::vector<owds::Shape>& visual_shapes);

  public:
    virtual ~Actor() noexcept;
    virtual void setup(const std::array<double, 3>& position,
                       const std::array<double, 4>& orientation) = 0;
    virtual void remove() = 0;

    /**
     * @param enabled Determines whether the actor should be influenced by the world (ie: move when it collides with other actors) or not.
     */
    virtual void setPhysicsEnabled(bool enabled) = 0;

    /**
     * @param enabled Determines whether the actor should influence the world (ie: can collide with other actors) or not.
     */
    virtual void setSimulationEnabled(bool enabled) = 0;

    /**
     * @param mass_kg Mass in kg(s), must be non-zero.
     */
    virtual void setMass(float mass_kg) = 0;

    /**
     * @param coefficient
     */
    virtual void setStaticFriction(float coefficient) = 0;

    /**
     * @param coefficient
     */
    virtual void setDynamicFriction(float coefficient) = 0;

    /**
     * @param coefficient De-penetration coefficient, increasing this will make the actor more bouncy.
     */
    virtual void setRestitution(float coefficient) = 0;

    /**
     * @param position Absolute position.
     * @param orientation Absolute rotation, in radians.
     */
    virtual void setPositionAndOrientation(const std::array<double, 3>& position, const std::array<double, 4>& orientation) = 0;

    virtual void setVelocity(const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity) = 0;

    /**
     * @return todo: document this
     */
    [[nodiscard]] virtual std::array<float, 16> getModelMatrix() const = 0;

    /**
     *
     * @return todo: document this
     */
    [[nodiscard]] virtual std::pair<std::array<double, 3>, std::array<double, 4>> getPositionAndOrientation() const = 0;

    // Each actor is associated with a non-zero, unique id.
    const std::size_t unique_id_{};
    const owds::Shape collision_shape_;
    const std::vector<owds::Shape> visual_shapes_;
  };
} // namespace owds

#endif // OWDS_COMMON_ACTOR_H
