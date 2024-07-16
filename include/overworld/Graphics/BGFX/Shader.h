#ifndef OWDS_BGFX_SHADER_H
#define OWDS_BGFX_SHADER_H

#include <string>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Graphics/BGFX/EmbeddedAssets.h"

namespace owds::bgfx {

  class Shader
  {
  public:
    Shader(const std::string& vs_name, const std::string& fs_name)
    {
      const auto type = ::bgfx::getRendererType();
      const auto vsh = ::bgfx::createEmbeddedShader(s_owds_embedded_shaders, type, vs_name.c_str());
      const auto fsh = ::bgfx::createEmbeddedShader(s_owds_embedded_shaders, type, fs_name.c_str());
      handle_ = ::bgfx::createProgram(vsh, fsh, true);
    }

    ~Shader()
    {
      ::bgfx::destroy(handle_);
    }

    void submit(::bgfx::ViewId id = 0)
    {
      ::bgfx::submit(id, handle_);
    }

  private:
    ::bgfx::ProgramHandle handle_;
  };

} // namespace owds::bgfx

#endif // OWDS_BGFX_SHADER_H