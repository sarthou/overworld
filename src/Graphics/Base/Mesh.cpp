#include "overworld/Graphics/Base/Mesh.h"

namespace owds {
  static std::size_t s_global_id_counter_ = 1;

  Mesh Mesh::create()
  {
    return Mesh(static_cast<Id>(s_global_id_counter_++));
  }

  Mesh::Mesh(const Id id) : id_(id)
  {}
} // namespace owds