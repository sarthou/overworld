#include "overworld/Graphics/Base/Mesh.h"

namespace owds {
  static std::size_t s_global_id_counter_ = 1;

  Mesh::Mesh() : id_(s_global_id_counter_++) {}

} // namespace owds