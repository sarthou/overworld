#ifndef OWDS_GRAPHICS_BASE_MODEL_H
#define OWDS_GRAPHICS_BASE_MODEL_H

#include <string>

#include "overworld/Graphics/Base/Mesh.h"
#include "overworld/Graphics/Base/Material.h"

namespace owds {
  class Model
  {
  public:
    enum class Id : std::size_t {};

    static Model create();

    explicit Model(Id id);

    // As for actors, each model is associated with a non-zero, unique id.
    const Id id_;

    std::string source_path_{};
    std::vector<owds::Mesh> meshes_{};
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_MODEL_H
