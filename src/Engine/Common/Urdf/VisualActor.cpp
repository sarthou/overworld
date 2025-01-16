#include "overworld/Engine/Common/Urdf/VisualActor.h"

#include <array>
#include <cassert>
#include <set>
#include <vector>

#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/World.h"

namespace owds {

  VisualActor::VisualActor(const std::vector<owds::Shape>& visual_shapes) : Actor(ShapeDummy(), visual_shapes)
  {
  }

  void VisualActor::setup(const std::array<double, 3>& position,
                          const std::array<double, 4>& orientation)
  {
    position_ = position;
    orientation_ = orientation;
  }

  void VisualActor::setPhysicsEnabled(bool enabled)
  {
    (void)enabled;
    assert(false && "[VisualActor] Physics cannot be enabled on visual actor.");
  }

  void VisualActor::setSimulationEnabled(bool enabled)
  {
    (void)enabled;
    assert(false && "[VisualActor] Simulation cannot be enabled on visual actor.");
  }

  void VisualActor::setMass(float mass_kg)
  {
    (void)mass_kg;
    assert(false && "[VisualActor] Mass cannot be set on visual actor.");
  }

  void VisualActor::setStaticFriction(float coefficient)
  {
    (void)coefficient;
    assert(false && "[VisualActor] Static friction cannot be set on visual actor.");
  }

  void VisualActor::setDynamicFriction(float coefficient)
  {
    (void)coefficient;
    assert(false && "[VisualActor] Dynamic friction cannot be set on visual actor.");
  }

  void VisualActor::setRestitution(float coefficient)
  {
    (void)coefficient;
    assert(false && "[VisualActor] Restitution cannot be set on visual actor.");
  }

  void VisualActor::setPositionAndOrientation(const std::array<double, 3>& position, const std::array<double, 4>& orientation)
  {
    position_ = position;
    orientation_ = orientation;
  }

  void VisualActor::setVelocity(const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity)
  {
    (void)linear_velocity;
    (void)angular_velocity;
    assert(false && "[VisualActor] Velocity cannot be set on visual actor.");
  }

  AABB_t VisualActor::getAABB()
  {
    assert(false && "[VisualActor] AABB cannot be get on visual actor.");
    return AABB_t();
  }

  AABB_t VisualActor::getLocalAABB()
  {
    assert(false && "[VisualActor] Local AABB cannot be get on visual actor.");
    return AABB_t();
  }

  std::array<float, 16> VisualActor::getModelMatrix() const
  {
    glm::mat4 translate = glm::translate(glm::mat4(1), glm::vec3(position_[0], position_[1], position_[2]));
    glm::mat4 rotate = glm::mat4_cast(glm::quat(glm::vec4(orientation_[0], orientation_[1], orientation_[2], orientation_[3])));
    glm::mat4 transform = translate * rotate;
    return FromM4(transform);
  }

  std::pair<std::array<double, 3>, std::array<double, 4>> VisualActor::getPositionAndOrientation() const
  {
    return {position_, orientation_};
  }

} // namespace owds