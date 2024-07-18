#ifndef OWDS_GRAPHICS_BGFX_MODELHANDLE_H
#define OWDS_GRAPHICS_BGFX_MODELHANDLE_H

#include <vector>

#include "overworld/Engine/Common/Models/Color.h"
#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Graphics/BGFX/MeshHandle.h"

namespace owds::bgfx {
  class ModelHandle
  {
  public:
    ::owds::Color color_rgba_{};
    float specular_;
    float shininess_;
    ::bgfx::TextureHandle tex_diffuse_{};
    ::bgfx::TextureHandle tex_specular_{};
    std::unordered_map<owds::Mesh::Id, owds::bgfx::MeshHandle> meshes_;
  };
} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_MODELHANDLE_H
