#include "overworld/Graphics/Base/Model.h"

namespace owds {
  static std::size_t s_global_id_counter_ = 1;

  Model Model::create()
  {
    return Model(s_global_id_counter_++);
  }

  Model::Model(const std::size_t id) : id_(id)
  {}
} // namespace owds