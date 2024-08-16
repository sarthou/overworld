#include "overworld/Engine/Graphics/GLFW/Window.h"

#include "overworld/Compat/ROS.h"
// should be first

#include <memory>
#include <string>

#include "overworld/Engine/Graphics/Common/WindowPlatformData.h"
#include "overworld/Engine/Graphics/OpenGL/Camera.h"
#include "overworld/Engine/Graphics/OpenGL/Renderer.h"

#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#include "stb_image.h"
#endif

// Should be last
#define GLFW_EXPOSE_NATIVE_X11
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

namespace owds {

  Window::Window(const std::string& name)
  {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
    // glfwWindowHint(GLFW_SAMPLES, 4);
    glfw_window_ = glfwCreateWindow(640, 480, name.c_str(), nullptr, nullptr);

    GLFWimage icons[1];
    std::string icon_path(owds::compat::owds_ros::getShareDirectory("overworld") + "/docs/images/overworld_light.png");
    icons[0].pixels = stbi_load(icon_path.c_str(), &icons[0].width, &icons[0].height, 0, 4); // rgba channels
    glfwSetWindowIcon(glfw_window_, 1, icons);
    stbi_image_free(icons[0].pixels);

    glfwMakeContextCurrent(glfw_window_);
    glfwSetWindowUserPointer(glfw_window_, this);
    glfwSetWindowSizeCallback(glfw_window_, [](GLFWwindow* window, const int width, const int height) {
      auto* const user_ptr = glfwGetWindowUserPointer(window);

      const auto& ctx = static_cast<Window*>(user_ptr);
      ctx->width_ = width;
      ctx->height_ = height;
      ctx->has_size_changed_ = true;
    });

    glfwSetKeyCallback(glfw_window_, [](GLFWwindow* window, const int key, const int scancode, const int action, const int mods) {
      (void)scancode;
      (void)mods;

      if(action == GLFW_REPEAT)
        return;

      auto* const user_ptr = glfwGetWindowUserPointer(window);
      const auto& ctx = static_cast<Window*>(user_ptr);

      if(ctx->camera_ != nullptr)
        ctx->camera_->processUserKeyboardInput(0.f, key, action == GLFW_PRESS);
    });

    glfwSetCursorPosCallback(glfw_window_, [](GLFWwindow* window, const double xpos, const double ypos) {
      auto* const user_ptr = glfwGetWindowUserPointer(window);
      const auto& ctx = reinterpret_cast<Window*>(user_ptr);

      if(ctx->camera_ != nullptr)
        ctx->camera_->processUserMouseInput(0.f, static_cast<float>(xpos), static_cast<float>(ypos));
    });

    glfwSetScrollCallback(glfw_window_, [](GLFWwindow* window, double xoffset, double yoffset) {
      auto* const user_ptr = glfwGetWindowUserPointer(window);
      const auto& ctx = reinterpret_cast<Window*>(user_ptr);

      if(ctx->camera_ != nullptr)
        ctx->camera_->processUserMouseScroll(0.f, static_cast<float>(xoffset), static_cast<float>(yoffset));
    });

    glfwSetMouseButtonCallback(glfw_window_, [](GLFWwindow* window, const int button, const int action, int mods) {
      (void)mods;

      if(action == GLFW_REPEAT)
        return;

      auto* const user_ptr = glfwGetWindowUserPointer(window);
      const auto& ctx = reinterpret_cast<Window*>(user_ptr);

      if(ctx->camera_ != nullptr)
        ctx->camera_->processUserMouseBtnInput(0.f, button, action == GLFW_PRESS);
    });
  }

  Window::~Window()
  {
    glfwDestroyWindow(glfw_window_);
    glfwTerminate();
  }

  WindowPlatformData Window::getPlatformData() const
  {
    return {
      reinterpret_cast<void*>(glfwGetX11Display()),
      reinterpret_cast<void*>(glfwGetX11Window(glfw_window_))};
  }

  void Window::doPollEvents(Renderer& renderer)
  {
    glfwPollEvents();

    camera_ = renderer.getRenderCamera();
    if(camera_ != nullptr)
      camera_->update();

    if(has_size_changed_)
    {
      has_size_changed_ = false;
      renderer.notifyResize(width_, height_);
    }
  }

  bool Window::isCloseRequested() const
  {
    return glfwWindowShouldClose(glfw_window_);
  }

  void Window::swapBuffer()
  {
    glfwSwapBuffers(glfw_window_);
  }

} // namespace owds