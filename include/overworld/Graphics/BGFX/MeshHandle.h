#ifndef OWDS_GRAPHICS_BGFX_MESH_H
#define OWDS_GRAPHICS_BGFX_MESH_H

#include "overworld/Graphics/BGFX/API.h"

namespace owds::bgfx {
  class MeshHandle
  {
  public:
    ::bgfx::VertexBufferHandle vbh_{};
    ::bgfx::IndexBufferHandle ibh_{};
  };
} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_MESH_H
