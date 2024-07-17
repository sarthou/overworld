#ifndef OWDS_COMMON_URDF_H
#define OWDS_COMMON_URDF_H

#include <string>
#include <unordered_map>

#include "overworld/Engine/Common/Models/Material.h"

namespace owds {
  class Actor;
  class Material;

  class Urdf
  {
  public:
    Urdf();
    ~Urdf() noexcept;

    std::string name_;
    std::unordered_map<std::string, owds::Actor*> links_;
    std::unordered_map<std::string, owds::Material> materials_;
  };
} // namespace owds

#endif // OWDS_COMMON_URDF_H