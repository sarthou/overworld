#ifndef OWDS_GRAPHICS_BASE_MODEL_H
#define OWDS_GRAPHICS_BASE_MODEL_H

#include <string>

#include "overworld/Graphics/Base/Mesh.h"

namespace owds {
  class Model
  {
  public:
    static Model create();

    explicit Model(std::size_t id);

    // As for actors, each model is associated with a non-zero, unique id.
    const std::size_t id_;

    std::string source_path_{};
    std::string texture_path_{};
    std::uint16_t texture_width_{};
    std::uint16_t texture_height_{};
    std::vector<owds::Mesh> meshes_{};
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_MODEL_H
