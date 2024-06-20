#include "overworld/Graphics/Base/Mesh.h"

namespace owds {
  static std::size_t s_global_id_counter_ = 1;

  Mesh Mesh::create()
  {
    return Mesh(s_global_id_counter_++);
  }

  Mesh::Mesh(const std::size_t id) : id_(id)
  {}
} // namespace owds