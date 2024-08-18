#include "overworld/Engine/Graphics/GLFW/Window.h"

#include "overworld/Compat/ROS.h"
// should be first

#include <memory>
#include <string>

#include "overworld/Engine/Graphics/Common/Camera.h"
#include "overworld/Engine/Graphics/Common/WindowPlatformData.h"
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

  void Window::init()
  {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
  }

  void Window::release()
  {
    glfwTerminate();
  }

  void Window::pollEvent()
  {
    glfwPollEvents();
  }

  Window::Window(const std::string& name)
  {
    // glfwWindowHint(GLFW_SAMPLES, 4);
    glfw_window_ = glfwCreateWindow(640, 480, name.c_str(), nullptr, nullptr);

    GLFWimage icons[1];
    std::string icon_path(owds::compat::owds_ros::getShareDirectory("overworld") + "/docs/images/overworld_light.png");
    icons[0].pixels = stbi_load(icon_path.c_str(), &icons[0].width, &icons[0].height, 0, 4); // rgba channels
    glfwSetWindowIcon(glfw_window_, 1, icons);
    stbi_image_free(icons[0].pixels);

    glfwSetWindowUserPointer(glfw_window_, this);
    glfwSetWindowSizeCallback(glfw_window_, [](GLFWwindow* glfw_window, const int width, const int height) {
      auto* const window = static_cast<Window*>(glfwGetWindowUserPointer(glfw_window));
      window->cam_mutex_.lock();
      window->camera_.setOutputResolution({(float)width, (float)height});
      window->cam_mutex_.unlock();
    });

    glfwSetKeyCallback(glfw_window_, [](GLFWwindow* glfw_window, const int key, const int scancode, const int action, const int mods) {
      (void)scancode;
      (void)mods;

      if(action == GLFW_REPEAT)
        return;

      auto* const window = static_cast<Window*>(glfwGetWindowUserPointer(glfw_window));
      window->cam_mutex_.lock();
      window->camera_.processUserKeyboardInput(0.f, key, action == GLFW_PRESS);
      window->cam_mutex_.unlock();
    });

    glfwSetCursorPosCallback(glfw_window_, [](GLFWwindow* glfw_window, const double xpos, const double ypos) {
      auto* const window = static_cast<Window*>(glfwGetWindowUserPointer(glfw_window));
      window->cam_mutex_.lock();
      window->camera_.processUserMouseInput(0.f, static_cast<float>(xpos), static_cast<float>(ypos));
      window->cam_mutex_.unlock();
    });

    glfwSetScrollCallback(glfw_window_, [](GLFWwindow* glfw_window, double xoffset, double yoffset) {
      auto* const window = static_cast<Window*>(glfwGetWindowUserPointer(glfw_window));
      window->cam_mutex_.lock();
      window->camera_.processUserMouseScroll(0.f, static_cast<float>(xoffset), static_cast<float>(yoffset));
      window->cam_mutex_.unlock();
    });

    glfwSetMouseButtonCallback(glfw_window_, [](GLFWwindow* glfw_window, const int button, const int action, int mods) {
      (void)mods;

      if(action == GLFW_REPEAT)
        return;

      auto* const window = static_cast<Window*>(glfwGetWindowUserPointer(glfw_window));
      window->cam_mutex_.lock();
      window->camera_.processUserMouseBtnInput(0.f, button, action == GLFW_PRESS);
      window->cam_mutex_.unlock();
    });
  }

  Window::~Window()
  {
    glfwDestroyWindow(glfw_window_);
  }

  void Window::makeCurrentContext() const
  {
    glfwMakeContextCurrent(glfw_window_);
  }

  WindowPlatformData Window::getPlatformData() const
  {
    return {
      reinterpret_cast<void*>(glfwGetX11Display()),
      reinterpret_cast<void*>(glfwGetX11Window(glfw_window_))};
  }

  void Window::doPollEvents(Renderer& renderer)
  {
    cam_mutex_.lock();
    camera_.update();
    camera_.finalize();
    renderer.setRenderCamera(camera_.getCamera());
    camera_.clearChanges();
    cam_mutex_.unlock();
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