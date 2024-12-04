#include "overworld/Engine/Common/Urdf/Urdf.h"

#include <cstddef>

#include "overworld/Engine/Common/Urdf/UrdfLoader.h"

namespace owds {

  static std::size_t u_global_id_counter = 1;

  Urdf::Urdf() : unique_id_(u_global_id_counter++)
  {}

  Urdf::~Urdf() noexcept = default;

} // namespace owds