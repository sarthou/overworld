#ifndef OWDS_GRAPHICS_GLFW3_CONTEXT_H
#define OWDS_GRAPHICS_GLFW3_CONTEXT_H

#define GLFW_EXPOSE_NATIVE_X11
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#include <cstdint>

namespace owds::glfw3 {
  class Context
  {
  public:
    std::uint32_t width_ = 640;
    std::uint32_t height_ = 480;
    bool has_size_changed_ = false;
    GLFWwindow* glfw_window_ = nullptr;
    owds::Camera* camera_ = nullptr;

    Context() = default;
    ~Context() noexcept = default;

    Context(const Context& other) = delete;
    Context& operator=(const Context& other) = delete;

    Context(Context&& other) = delete;
    Context& operator=(Context&& other) = delete;
  };
} // namespace owds::glfw3

#endif // OWDS_GRAPHICS_GLFW3_CONTEXT_H