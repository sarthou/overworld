#ifndef OWDS_PHYSICS_PHYSX_URDF_H
#define OWDS_PHYSICS_PHYSX_URDF_H

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "overworld/Engine/Common/Urdf/Urdf.h"
#include "overworld/Engine/Physics/PhysX/API.h"

namespace owds::physx {

  class Context;

  class Urdf : public owds::Urdf
  {
  public:
    Urdf(owds::physx::Context& ctx);

    virtual ~Urdf() noexcept override = default;

    Urdf(const Urdf& other) = delete;
    Urdf& operator=(const Urdf& other) = delete;

    Urdf(Urdf&& other) noexcept = delete;
    Urdf& operator=(Urdf&& other) = delete;

    void setup() override;
    void finish() override;

    void addLink(const std::string& parent,
                 const std::string& link_name,
                 const glm::vec3& origin_translation,
                 const glm::vec3& origin_rotation,
                 const owds::Shape& collision_shape,
                 const std::vector<owds::Shape>& visual_shapes) override;

    void addJoint(const urdf::Joint_t& joint) override;

    void remove() override;

    void setPhysicsEnabled(bool enabled) override;

    // void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 4>& orientation);

    // std::pair<std::array<float, 3>, std::array<float, 3>> getPositionAndOrientation() const override {};

  protected:
    owds::physx::Context& ctx_;

    PxPtr<::physx::PxArticulationReducedCoordinate> px_articulation_;

    std::unordered_map<std::string, ::physx::PxArticulationLink*> px_links_;
    std::unordered_map<std::string, ::physx::PxArticulationJointReducedCoordinate*> px_joints_;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_URDF_H