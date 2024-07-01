#include "overworld/Graphics/BGFX/Context.h"
#include "overworld/Graphics/BGFX/Camera.h"

namespace owds::bgfx {
  Context::Context() = default;

  Context::~Context()
  {
    for(auto& [name, handle] : loaded_programs_)
    {
      ::bgfx::destroy(handle);
    }
  }
} // namespace owds::bgfx