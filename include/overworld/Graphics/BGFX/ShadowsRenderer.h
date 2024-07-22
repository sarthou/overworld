#ifndef OWDS_GRAPHICS_BGFX_SHADOWSRENDERER_H
#define OWDS_GRAPHICS_BGFX_SHADOWSRENDERER_H

#include <cstdint>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Graphics/BGFX/Camera.h"

namespace owds::bgfx {

  class ShadowsRenderer
  {
  public:
    ShadowsRenderer(::bgfx::ViewId view_id);

    bool initialize();

    ::bgfx::TextureHandle texture_handle_;
    Camera camera_;

    unsigned long state_;

  private:
    std::uint32_t shadow_map_size_;

    ::bgfx::ViewId view_id_;
    ::bgfx::FrameBufferHandle frame_buffer_;
  };

} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_SHADOWSRENDERER_H