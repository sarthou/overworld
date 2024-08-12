#include "overworld/Engine/Common/Urdf/Joints/Joint.h"

#include "overworld/Engine/Common/Urdf/Actor.h"

namespace owds {
  Joint::Joint(const owds::JointLocation& location)
    : location_(location)
  {
    location_.parent_.joints_.insert(this);
    location_.child_.joints_.insert(this);
  }

  Joint::~Joint() noexcept
  {
    location_.parent_.joints_.erase(this);
    location_.child_.joints_.erase(this);
  }

} // namespace owds