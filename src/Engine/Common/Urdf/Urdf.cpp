#include "overworld/Engine/Common/Urdf/Urdf.h"

#include <cstddef>
#include <string>

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

} // namespace owds