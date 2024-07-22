#include "overworld/Graphics/BGFX/ShadowsRenderer.h"

namespace owds::bgfx {

  ShadowsRenderer::ShadowsRenderer(::bgfx::ViewId view_id) : shadow_map_size_(512),
                                                             view_id_(view_id)

  {
    state_ = BGFX_STATE_WRITE_Z |
             BGFX_STATE_DEPTH_TEST_LESS |
             BGFX_STATE_CULL_CCW |
             BGFX_STATE_MSAA;
  }

  bool ShadowsRenderer::initialize()
  {
    ::bgfx::TextureHandle textures[] =
      {
        ::bgfx::createTexture2D(
          shadow_map_size_, shadow_map_size_, false, 1, ::bgfx::TextureFormat::D16, BGFX_TEXTURE_RT | BGFX_SAMPLER_COMPARE_LEQUAL),
      };

    texture_handle_ = textures[0];
    frame_buffer_ = ::bgfx::createFrameBuffer(1, textures, true);

    camera_.setViewId(view_id_);
    camera_.setCameraView(owds::CameraView_e::segmented_view);
    camera_.setProjection(owds::CameraProjection_e::orthographic);
    camera_.setFieldOfView(80.f);
    camera_.setOutputResolution({(float)shadow_map_size_, (float)shadow_map_size_});
    camera_.finalize();

    ::bgfx::setViewFrameBuffer(view_id_, frame_buffer_);
    ::bgfx::setViewClear(view_id_, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f, 0);

    return true;
  }

} // namespace owds::bgfx