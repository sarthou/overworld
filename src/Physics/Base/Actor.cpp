#include "overworld/Physics/Base/Actor.h"

#include "overworld/Physics/Base/Joints/Joint.h"

namespace owds {
  static std::size_t s_global_id_counter = 1;

  Actor::Actor(const owds::Shape& shape)
    : unique_id_(s_global_id_counter++),
      shape_(shape)
  {}

  Actor::~Actor()
  {
    for(const auto& joint : joints_)
    {
      joint->remove();
    }
  }
} // namespace owds