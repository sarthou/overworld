#include "overworld/BasicTypes/BodyPart.h"

namespace owds {

  BodyPart::BodyPart(const std::string& id, bool is_true_id) : Entity(id, is_true_id)
  {
    type_ = BODY_PART_UNKNOW;
  }

} // namespace owds
