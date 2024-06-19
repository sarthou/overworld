#ifndef OWDS_PHYSICS_BASE_ACTOR_H
#define OWDS_PHYSICS_BASE_ACTOR_H

#include <array>
#include <set>

#include "overworld/Shapes/Shape.h"

namespace owds {
  class Joint;

  class Actor
  {
  protected:
    explicit Actor(const owds::Shape& shape);

  public:
    virtual ~Actor() noexcept;
    virtual void setup() = 0;
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
    virtual void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation) = 0;

    [[nodiscard]] virtual std::array<float, 16> getModelMatrix() const = 0;
    [[nodiscard]] virtual std::pair<std::array<float, 3>, std::array<float, 3>> getPositionAndOrientation() const = 0;

    // Each actor is associated with a non-zero, unique id.
    const std::size_t unique_id_{};

    owds::Shape shape_;
    std::set<owds::Joint*> joints_;
  };
} // namespace owds

#endif // OWDS_PHYSICS_BASE_ACTOR_H
