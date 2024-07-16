#ifndef OWDS_GRAPHICS_BASE_WINDOW_H
#define OWDS_GRAPHICS_BASE_WINDOW_H

#include "overworld/Graphics/Base/WindowPlatformData.h"

namespace owds {
  class Renderer;

  class Window
  {
  public:
    virtual ~Window() = default;

    [[nodiscard]] virtual WindowPlatformData getPlatformData() const = 0;
    virtual void doPollEvents(owds::Renderer& renderer) = 0;
    [[nodiscard]] virtual bool isCloseRequested() const = 0;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_WINDOW_H
