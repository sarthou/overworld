#ifndef OWDS_GRAPHICS_GLFW3_WINDOW_H
#define OWDS_GRAPHICS_GLFW3_WINDOW_H

#include <memory>

#include "overworld/Graphics/Base/Window.h"

namespace owds::glfw3 {
  class Context;

  class Window final : public owds::Window
  {
  public:
    Window();
    ~Window() override;

    [[nodiscard]] WindowPlatformData getPlatformData() const override;

    void doPollEvents(owds::Renderer& renderer) override;
    [[nodiscard]] bool isCloseRequested() const override;

  protected:
    std::unique_ptr<owds::glfw3::Context> ctx_;
  };
} // namespace owds::glfw3

#endif // OWDS_GRAPHICS_GLFW3_WINDOW_H
