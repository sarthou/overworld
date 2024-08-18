#ifndef OWDS_GRAPHICS_GLFW3_WINDOW_H
#define OWDS_GRAPHICS_GLFW3_WINDOW_H

#include <cstdint>
#include <mutex>
#include <string>

#include "overworld/Engine/Graphics/Common/WindowPlatformData.h"
#include "overworld/Engine/Graphics/GLFW/CameraUpdater.h"

class GLFWwindow;

namespace owds {
  class Renderer;

  class Window
  {
  public:
    static void init();
    static void release();
    static void pollEvent();

    Window(const std::string& name = "Overworld");
    ~Window();

    void makeCurrentContext() const;

    WindowPlatformData getPlatformData() const;

    void doPollEvents(owds::Renderer& renderer);
    bool isCloseRequested() const;
    void swapBuffer();

    CameraUpdater& getCamera() { return camera_; }
    Camera* getUpdatedCamera() { return camera_.getCamera(); }

  protected:
    std::uint32_t width_ = 640;
    std::uint32_t height_ = 480;
    bool has_size_changed_ = false;
    GLFWwindow* glfw_window_ = nullptr;
    CameraUpdater camera_;
    std::mutex cam_mutex_;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_GLFW3_WINDOW_H
