#include "overworld/Engine/Common/Urdf/Urdf.h"

#include <cstddef>
#include <string>

#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"

namespace owds {

  static std::size_t u_global_id_counter = 1;

  Urdf::Urdf() : unique_id_(u_global_id_counter++)
  {}

  Urdf::~Urdf() noexcept = default;

  bool Urdf::setJointState(const std::string& joint_name, double position, double velocity)
  {
    auto joint_it = joints_.find(joint_name);
    if(joint_it != joints_.end())
    {
      joint_it->second->setPosition(position);
      joint_it->second->setVelocity(velocity);
      return true;
    }
    else
      return false;
  }

  int Urdf::getLinkId(const std::string& link_name)
  {
    auto it = links_.find(link_name);
    if(it != links_.end())
      return it->second->unique_id_;
    else
      return -1;
  }

  void Urdf::setMass(int link_index, double mass_kg)
  {
    auto link_it = id_links_.find(link_index);
    if(link_it != id_links_.end())
      link_it->second->setMass((float)mass_kg);
  }

  void Urdf::setStaticFriction(int link_index, double friction)
  {
    auto link_it = id_links_.find(link_index);
    if(link_it != id_links_.end())
      link_it->second->setStaticFriction((float)friction);
  }

  void Urdf::setDynamicFriction(int link_index, double friction)
  {
    auto link_it = id_links_.find(link_index);
    if(link_it != id_links_.end())
      link_it->second->setDynamicFriction((float)friction);
  }

  void Urdf::setRestitution(int link_index, double restitution)
  {
    auto link_it = id_links_.find(link_index);
    if(link_it != id_links_.end())
      link_it->second->setRestitution((float)restitution);
  }

} // namespace owds