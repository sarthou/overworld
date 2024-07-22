#ifndef OWDS_GRAPHICS_GLFW3_WINDOW_H
#define OWDS_GRAPHICS_GLFW3_WINDOW_H

#include <cstdint>
#include <string>

#include "overworld/Engine/Graphics/Common/WindowPlatformData.h"

class GLFWwindow;

namespace owds {
  class Camera;
  class Renderer;

  class Window
  {
  public:
    Window(const std::string& name = "Overworld");
    ~Window();

    WindowPlatformData getPlatformData() const;

    void doPollEvents(owds::Renderer& renderer);
    bool isCloseRequested() const;
    void swapBuffer();

  protected:
    std::uint32_t width_ = 640;
    std::uint32_t height_ = 480;
    bool has_size_changed_ = false;
    GLFWwindow* glfw_window_ = nullptr;
    Camera* camera_ = nullptr;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_GLFW3_WINDOW_H
