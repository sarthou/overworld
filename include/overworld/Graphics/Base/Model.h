#ifndef OWDS_GRAPHICS_BASE_MODEL_H
#define OWDS_GRAPHICS_BASE_MODEL_H

#include <string>

#include "overworld/Graphics/Base/Mesh.h"

namespace owds {
  class Model
  {
  public:
    std::string source_path_{};
    std::vector<owds::Mesh> meshes_{};
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_MODEL_H
