#include "overworld/Engine/Common/Urdf/Joint.h"

#include <cstddef>

#include "overworld/Engine/Common/Urdf/UrdfLoader.h"

namespace owds {

  static std::size_t j_global_id_counter = 1;

  Joint::Joint(urdf::JointType_e type, int axis,
               int direction) : unique_id_(j_global_id_counter++),
                                type_(type),
                                axis_(axis),
                                direction_(direction),
                                has_limits_(false)
  {}

  void Joint::setLimits(double low, double high)
  {
    has_limits_ = true;
    limit_low_ = low;
    limit_high_ = high;
  }

} // namespace owds