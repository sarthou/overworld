#include "overworld/Graphics/GLFW3/Window.h"

#include "overworld/Graphics/Base/Renderer.h"
#include "overworld/Graphics/GLFW3/Context.h"

namespace owds::glfw3 {
  Window::Window() : ctx_(std::make_unique<owds::glfw3::Context>())
  {
    glfwInit();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    ctx_->glfw_window_ = glfwCreateWindow(640, 480, "Default GLFW3 window", nullptr, nullptr);

    glfwSetWindowUserPointer(ctx_->glfw_window_, this);
    glfwSetWindowSizeCallback(ctx_->glfw_window_, [](GLFWwindow* window, int width, int height) {
      const auto& ctx_ = reinterpret_cast<owds::glfw3::Window*>(glfwGetWindowUserPointer(window))->ctx_;
      ctx_->width_ = width;
      ctx_->height_ = height;
      ctx_->has_size_changed_ = true;
    });
  }

  Window::~Window()
  {
    glfwDestroyWindow(ctx_->glfw_window_);
    glfwTerminate();
  }

  WindowPlatformData Window::getPlatformData() const
  {
    return {
      reinterpret_cast<void*>(glfwGetX11Display()),
      reinterpret_cast<void*>(glfwGetX11Window(ctx_->glfw_window_))};
  }

  void Window::doPollEvents(owds::Renderer& renderer)
  {
    glfwPollEvents();

    if(ctx_->has_size_changed_)
    {
      ctx_->has_size_changed_ = false;
      renderer.notifyResize(ctx_->width_, ctx_->height_);
    }
  }

  bool Window::isCloseRequested() const
  {
    return glfwWindowShouldClose(ctx_->glfw_window_);
  }
} // namespace owds::glfw3