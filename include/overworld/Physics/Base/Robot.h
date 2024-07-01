#ifndef OWDS_PHYSICS_BASE_ROBOT_H
#define OWDS_PHYSICS_BASE_ROBOT_H

#include <string>
#include <unordered_map>
#include "overworld/Graphics/Base/Material.h"

namespace owds {
  class Actor;
  class Material;

  class Robot
  {
  public:
    Robot();
    ~Robot() noexcept;

    std::string name_;
    std::unordered_map<std::string, owds::Actor*> links_;
    std::unordered_map<std::string, owds::Material> materials_;
  };
} // namespace owds

#endif // OWDS_PHYSICS_BASE_ROBOT_H