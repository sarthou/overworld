#include "overworld/Graphics/BGFX/Camera.h"

#include <cstdio>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Helper/BitCast.h"
#include "overworld/Helper/GlmMath.h"

#include <GLFW/glfw3.h>

namespace owds::bgfx {
  Camera::Camera(owds::World& world)
    : currently_viewed_world_(world) {}

  void Camera::updateViewMatrix()
  {
    view_matrix_ = FromM4(glm::lookAt(
      world_eye_position_,
      world_eye_position_ + world_eye_front_,
      world_eye_up_));

    ::bgfx::setViewTransform(id_, view_matrix_.data(), proj_matrix_.data());
  }

  void Camera::updateProjectionMatrix()
  {
    // todo: check if everything is setup correctly
    switch(projection_type_)
    {
    case owds::CameraProjection_e::perspective:
    {
      constexpr auto near = 0.1f;
      constexpr auto far = 100.f;
      const auto aspect_ratio = view_dimensions_[0] / view_dimensions_[1];

      if(::bgfx::getCaps()->homogeneousDepth)
      {
        proj_matrix_ = FromM4(glm::perspectiveLH_NO<float>(field_of_view_, aspect_ratio, near, far));
      }
      else
      {
        proj_matrix_ = FromM4(glm::perspectiveLH_ZO<float>(field_of_view_, aspect_ratio, near, far));
      }
      break;
    }
    case owds::CameraProjection_e::orthographic:
    {
      if(::bgfx::getCaps()->homogeneousDepth)
      {
        proj_matrix_ = FromM4(glm::orthoLH_NO<float>(0, 0, view_dimensions_[0], view_dimensions_[1], -1000.0f, 1000.0f));
      }
      else
      {
        proj_matrix_ = FromM4(glm::orthoLH_ZO<float>(0, 0, view_dimensions_[0], view_dimensions_[1], -1000.0f, 1000.0f));
      }
      break;
    }
    }

    ::bgfx::setViewTransform(id_, view_matrix_.data(), proj_matrix_.data());
  }

  void Camera::updateMatrices()
  {
    updateViewMatrix();
    updateProjectionMatrix();
  }

  void Camera::setCameraView(const owds::CameraView_e view_type)
  {
    view_type_ = view_type;
  }

  void Camera::setProjection(const owds::CameraProjection_e proj)
  {
    projection_type_ = proj;
  }

  void Camera::setFieldOfView(const float fov)
  {
    field_of_view_ = glm::radians(fov);
  }

  void Camera::setOutputAA(const owds::ViewAntiAliasing_e aa_setting)
  {
    (void)aa_setting; // todo
  }

  void Camera::setOutputFPS(const std::uint64_t desired_target_fps)
  {
    (void)desired_target_fps; // todo
  }

  void Camera::setOutputResolution(const std::array<float, 2>& resolution)
  {
    view_dimensions_[0] = resolution[0];
    view_dimensions_[1] = resolution[1];
    ::bgfx::setViewRect(id_, 0, 0, static_cast<uint16_t>(view_dimensions_[0]), static_cast<uint16_t>(view_dimensions_[1]));
  }

  void Camera::setOutputClearColor(const owds::Color clear_color)
  {
    ::bgfx::setViewClear(id_, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, owds::BitCast<std::uint32_t>(clear_color), 1.0f, 0);
  }

  void Camera::setCaptureCallback(std::function<void(const std::vector<owds::Color>&)> callback)
  {
    (void)callback; // todo
  }

  void Camera::finalize()
  {
    updateViewMatrix();
  }

  void Camera::setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation)
  {
    (void)position;    // todo
    (void)orientation; // todo
  }

  void Camera::setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position)
  {
    world_eye_position_ = ToV3(eye_position);
    world_eye_front_ = ToV3(dst_position) - ToV3(eye_position);
    updateViewMatrix();
  }

  void Camera::recomputeDirectionVector()
  {
    world_eye_front_.x = glm::cos(view_angles_.x) * glm::cos(view_angles_.y);
    world_eye_front_.y = glm::sin(view_angles_.x) * glm::cos(view_angles_.y);
    world_eye_front_.z = glm::sin(view_angles_.y);

    const auto d90 = glm::pi<float>() / 2.f;
    world_eye_right_.x = glm::cos(view_angles_.x + d90);
    world_eye_right_.y = glm::sin(view_angles_.x + d90);
    world_eye_right_.z = 0.f;
  }

  void Camera::processUserKeyboardInput(const float delta_time, const int key, const bool is_down)
  {
    switch (key)
    {
    case GLFW_KEY_F1: show_debug_stats_ = is_down; break;
    case GLFW_KEY_F2: render_collision_models_ = is_down; break;
    case GLFW_KEY_UP:    key_state_front_ = is_down; break;
    case GLFW_KEY_DOWN:  key_state_down_  = is_down; break;
    case GLFW_KEY_LEFT:  key_state_left_  = is_down; break;
    case GLFW_KEY_RIGHT: key_state_right_ = is_down; break;
    }

    (void)delta_time;
    (void)key;
    (void)is_down;
  }

  void Camera::processUserMouseBtnInput(const float delta_time, const char btn, const bool is_down)
  {
    mouse_btn_states_.set(btn, is_down);

    switch(btn)
    {
    case 1:
    {
      if (is_down)
      {
        mouse_drag_start_position_ = mouse_current_position;
      }

      is_dragging_mouse_ = is_down;
      break;
    }
    default:;
    }

    (void)delta_time;
    (void)btn;
    (void)is_down;
  }

  void Camera::processUserMouseInput(const float delta_time, const float x, const float y)
  {
    (void)delta_time;
    mouse_current_position = {x, y};

    if (mouse_btn_states_[1])
    {
      const auto delta = (mouse_current_position - mouse_drag_start_position_) * 0.001f;
      mouse_drag_start_position_ = mouse_current_position;

      view_angles_.x += delta.x;
      view_angles_.y -= delta.y;

      recomputeDirectionVector();
    }
  }

  void Camera::update()
  {
    if (key_state_front_) world_eye_position_ += world_eye_front_ * 0.1f;
    if (key_state_down_) world_eye_position_ -= world_eye_front_ * 0.1f;
    if (key_state_right_) world_eye_position_ += world_eye_right_ * 0.1f;
    if (key_state_left_) world_eye_position_ -= world_eye_right_ * 0.1f;

    // world_eye_right_
  }

} // namespace owds::bgfx