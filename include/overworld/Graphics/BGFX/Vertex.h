#ifndef OWDS_VERTEX_H
#define OWDS_VERTEX_H

#include <bgfx/bgfx.h>

#include "overworld/Engine/Common/Models/Vertex.h"

namespace owds::bgfx {
  class Vertex : public owds::Vertex
  {
  public:
    static ::bgfx::VertexLayout getMSLayout();
  };
} // namespace owds::bgfx

#endif // OWDS_VERTEX_H
