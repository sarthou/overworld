#include "overworld/Graphics/Base/Model.h"

namespace owds {
  static std::size_t s_global_id_counter_ = 1;

  Model Model::create()
  {
    return Model(static_cast<Id>(s_global_id_counter_++));
  }

  Model::Model(const Id id) : id_(id)
  {}
} // namespace owds