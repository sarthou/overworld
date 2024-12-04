#ifndef OWDS_COMMON_URDF_H
#define OWDS_COMMON_URDF_H

#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "overworld/Engine/Common/Models/Material.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"

namespace owds {
  class Actor;
  class Joint;

  class Urdf
  {
  public:
    Urdf();
    virtual ~Urdf() noexcept;

    virtual void setup() = 0;
    virtual void finish() = 0;

    virtual void addLink(const std::string& parent,
                         const std::string& link_name,
                         const glm::vec3& origin_translation,
                         const glm::vec3& origin_rotation,
                         const owds::Shape& collision_shape,
                         const std::vector<owds::Shape>& visual_shapes) = 0;

    virtual void addJoint(const urdf::Joint_t& joint) = 0;

    virtual void remove() = 0;

    virtual void setPhysicsEnabled(bool enabled) = 0;

    // Each urdf is associated with a non-zero, unique id.
    const std::size_t unique_id_{};

    std::string name_;
    owds::Actor* root_actor_;
    std::unordered_map<std::string, owds::Actor*> links_;
    std::unordered_map<std::string, owds::Joint*> joints_;
    std::unordered_map<std::string, owds::Material> materials_;
  };
} // namespace owds

#endif // OWDS_COMMON_URDF_H