#include "overworld/Engine/Common/Urdf/Actor.h"

#include <cstddef>
#include <vector>

#include "overworld/Engine/Common/Shapes/Shape.h"

namespace owds {

  static std::size_t s_global_id_counter = 1;

  Actor::Actor(const owds::Shape& collision_shape,
               const std::vector<owds::Shape>& visual_shapes) : unique_id_(s_global_id_counter++),
                                                                collision_shape_(collision_shape),
                                                                visual_shapes_(visual_shapes)
  {}

  Actor::~Actor() noexcept = default;

} // namespace owds