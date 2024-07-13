#include "overworld/Graphics/GLFW3/Window.h"

#include <overworld/Graphics/Base/Camera.h>

#include "overworld/Graphics/Base/Renderer.h"
#include "overworld/Graphics/GLFW3/Context.h"

namespace owds::glfw3 {
  Window::Window() : ctx_(std::make_unique<owds::glfw3::Context>())
  {
    glfwInit();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    ctx_->glfw_window_ = glfwCreateWindow(640, 480, "Default GLFW3 window", nullptr, nullptr);

    glfwSetWindowUserPointer(ctx_->glfw_window_, this);
    glfwSetWindowSizeCallback(ctx_->glfw_window_, [](GLFWwindow* window, const int width, const int height) {
      const auto user_ptr = glfwGetWindowUserPointer(window);

      const auto& ctx_ = static_cast<owds::glfw3::Window*>(user_ptr)->ctx_;
      ctx_->width_ = width;
      ctx_->height_ = height;
      ctx_->has_size_changed_ = true;
    });

    glfwSetKeyCallback(ctx_->glfw_window_, [](GLFWwindow* window, const int key, const int scancode, const int action, const int mods) {
      (void)scancode;
      (void)mods;

      const auto user_ptr = glfwGetWindowUserPointer(window);

      if(action == GLFW_REPEAT)
      {
        return;
      }

      const auto& ctx_ = static_cast<owds::glfw3::Window*>(user_ptr)->ctx_;

      if(ctx_->cached_camera_refs_.empty())
      {
        return;
      }

      auto& camera = ctx_->cached_camera_refs_[ctx_->active_camera_index].get();

      camera.processUserKeyboardInput(0.f, key, action == GLFW_PRESS);
    });

    glfwSetCursorPosCallback(ctx_->glfw_window_, [](GLFWwindow* window, const double xpos, const double ypos) {
      const auto user_ptr = glfwGetWindowUserPointer(window);

      const auto& ctx_ = reinterpret_cast<owds::glfw3::Window*>(user_ptr)->ctx_;

      if(ctx_->cached_camera_refs_.empty())
      {
        return;
      }

      auto& camera = ctx_->cached_camera_refs_[ctx_->active_camera_index].get();

      camera.processUserMouseInput(0.f, static_cast<float>(xpos), static_cast<float>(ypos));
    });

    glfwSetScrollCallback(ctx_->glfw_window_, [](GLFWwindow* window, double xoffset, double yoffset) {
      const auto user_ptr = glfwGetWindowUserPointer(window);

      const auto& ctx_ = reinterpret_cast<owds::glfw3::Window*>(user_ptr)->ctx_;

      if(ctx_->cached_camera_refs_.empty())
      {
        return;
      }

      auto& camera = ctx_->cached_camera_refs_[ctx_->active_camera_index].get();

      camera.processUserMouseScroll(0.f, static_cast<float>(xoffset), static_cast<float>(yoffset));
    });

    glfwSetMouseButtonCallback(ctx_->glfw_window_, [](GLFWwindow* window, const int button, const int action, int mods) {
      (void)mods;

      const auto user_ptr = glfwGetWindowUserPointer(window);

      if(action == GLFW_REPEAT)
      {
        return;
      }

      const auto& ctx_ = reinterpret_cast<owds::glfw3::Window*>(user_ptr)->ctx_;

      if(ctx_->cached_camera_refs_.empty())
      {
        return;
      }

      auto& camera = ctx_->cached_camera_refs_[ctx_->active_camera_index].get();

      camera.processUserMouseBtnInput(0.f, button, action == GLFW_PRESS);
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

    ctx_->cached_camera_refs_ = renderer.getCameras();

    for(auto& cam : ctx_->cached_camera_refs_)
    {
      cam.get().update();
    }

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