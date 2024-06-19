#include "overworld/Graphics/BGFX/Vertex.h"

namespace owds::bgfx {
  ::bgfx::VertexLayout Vertex::getMSLayout()
  {
    static ::bgfx::VertexLayout ms_layout = []() {
      ::bgfx::VertexLayout ms_layout;

      ms_layout.begin()
        .add(::bgfx::Attrib::Position, 3, ::bgfx::AttribType::Float)
        .add(::bgfx::Attrib::Normal, 3, ::bgfx::AttribType::Float)
        .add(::bgfx::Attrib::TexCoord0, 2, ::bgfx::AttribType::Float)
        .add(::bgfx::Attrib::Color0, 4, ::bgfx::AttribType::Uint8, true)
        .end();

      return ms_layout;
    }();
    return ms_layout;
  }
} // namespace owds::bgfx