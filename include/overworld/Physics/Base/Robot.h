#ifndef OWDS_PHYSICS_BASE_ROBOT_H
#define OWDS_PHYSICS_BASE_ROBOT_H

#include <string>
#include <unordered_map>
#include <functional>

namespace owds {
  class Actor;

  class Robot
  {
  public:
    std::string name_;
    std::unordered_map<std::string, std::reference_wrapper<Actor>> links_;
  };
} // namespace owds

#endif // OWDS_PHYSICS_BASE_ROBOT_H