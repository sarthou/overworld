#ifndef OWDS_GRAPHICS_BASE_MESH_H
#define OWDS_GRAPHICS_BASE_MESH_H

#include <vector>

#include "overworld/Graphics/Base/Vertex.h"

namespace owds {
  class Mesh
  {
  public:
    enum class Id : std::size_t {};

    static Mesh create();

    explicit Mesh(Id id);

    // As for actors, each mesh is associated with a non-zero, unique id.
    const Id id_;

    std::string name_ {};
    std::vector<owds::Vertex> vertices_{};
    std::vector<std::uint32_t> indices_{};
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_MESH_H
