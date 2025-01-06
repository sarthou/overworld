#ifndef OWDS_COMMON_URDF_H
#define OWDS_COMMON_URDF_H

#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "overworld/Engine/Common/Models/Material.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/Joint.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"

namespace owds {
  class Actor;

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

    virtual size_t getNumJoints() = 0;

    virtual std::pair<std::array<double, 3>, std::array<double, 4>> getPositionAndOrientation() = 0;
    virtual void setPositionAndOrientation(const std::array<double, 3>& position, const std::array<double, 4>& orientation) = 0;
    virtual void setVelocity(const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity) = 0;
    bool setJointState(const std::string& joint_name, double position, double velocity);

    int getLinkId(const std::string& link_name);

    void setMass(int link_index, double mass_kg);
    void setStaticFriction(int link_index, double friction);
    void setDynamicFriction(int link_index, double friction);
    void setRestitution(int link_index, double restitution);

    // Each urdf is associated with a non-zero, unique id.
    const std::size_t unique_id_{};

    std::string name_;
    owds::Actor* root_actor_;
    std::unordered_map<std::string, owds::Actor*> links_;
    std::unordered_map<size_t, owds::Actor*> id_links_;
    std::unordered_map<std::string, owds::Joint*> joints_;
    std::unordered_map<std::string, owds::Material> materials_;
  };
} // namespace owds

#endif // OWDS_COMMON_URDF_H