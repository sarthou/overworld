#include "overworld/Engine/Graphics/OpenGL/Camera.h"

#include "glad/glad.h"

// first
#include <GLFW/glfw3.h>
#include <array>
#include <cstdint>
#include <cstdio>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "overworld/Engine/Graphics/Common/CameraProjection.h"
#include "overworld/Engine/Graphics/Common/CameraView.h"
#include "overworld/Engine/Graphics/Common/ViewAntiAliasing.h"
#include "overworld/Helper/GlmMath.h"

namespace owds {

  void Camera::updateViewMatrix()
  {
    view_matrix_ = glm::lookAt(world_eye_position_,
                               world_eye_position_ + world_eye_front_,
                               world_eye_up_);
  }

  void Camera::updateProjectionMatrix()
  {
    switch(projection_type_)
    {
    case owds::CameraProjection_e::perspective:
    {
      constexpr auto near = 0.1f;
      constexpr auto far = 100.f;
      const auto aspect_ratio = view_dimensions_[0] / view_dimensions_[1];

      proj_matrix_ = glm::perspective(field_of_view_, aspect_ratio, near, far);

      break;
    }
    case owds::CameraProjection_e::orthographic:
    {
      constexpr auto near = 0.1f;
      constexpr auto far = 100.f;
      proj_matrix_ = glm::ortho(-view_dimensions_[0] / 2, view_dimensions_[0] / 2,
                                -view_dimensions_[1] / 2, view_dimensions_[1] / 2,
                                near, far);
      break;
    }
    }
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
    aa_setting_ = aa_setting;
  }

  void Camera::setOutputFPS(const std::uint64_t desired_target_fps)
  {
    (void)desired_target_fps; // todo
  }

  void Camera::setOutputResolution(const std::array<float, 2>& resolution)
  {
    view_dimensions_[0] = resolution[0];
    view_dimensions_[1] = resolution[1];
  }

  void Camera::finalize()
  {
    recomputeDirectionVector();
    updateMatrices();
  }

  void Camera::setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation)
  {
    // to verify
    world_eye_position_ = ToV3(position);
    view_angles_.x = orientation[2]; // yaw
    view_angles_.y = orientation[1]; // pitch
    recomputeDirectionVector();
  }

  void Camera::setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position)
  {
    world_eye_position_ = ToV3(eye_position);
    world_eye_front_ = ToV3(dst_position) - world_eye_position_;

    const auto xy = glm::sqrt(glm::pow(world_eye_front_.x, 2) + glm::pow(world_eye_front_.y, 2));

    view_angles_.x = glm::degrees(glm::atan(world_eye_front_.y, world_eye_front_.x));
    view_angles_.y = glm::degrees(glm::atan(world_eye_front_.z, static_cast<float>(xy)));

    recomputeDirectionVector();
    updateViewMatrix();
  }

  void Camera::recomputeDirectionVector()
  {
    glm::vec3 world_up(0.0f, 0.0f, 1.0f);

    glm::vec3 front;
    front.x = cos(glm::radians(view_angles_.x)) * cos(glm::radians(view_angles_.y));
    front.y = sin(glm::radians(view_angles_.x)) * cos(glm::radians(view_angles_.y));
    front.z = sin(glm::radians(view_angles_.y));

    world_eye_front_ = glm::normalize(front);
    world_eye_right_ = glm::normalize(glm::cross(world_eye_front_, world_up)); // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
    world_eye_up_ = glm::normalize(glm::cross(world_eye_right_, world_eye_front_));
  }

  void Camera::processUserKeyboardInput(const float delta_time, const int key, const bool is_down)
  {
    switch(key)
    {
    case GLFW_KEY_F1: show_debug_stats_ = is_down; break;
    case GLFW_KEY_F2: render_collision_models_ = is_down; break;
    case GLFW_KEY_W:
    case GLFW_KEY_UP: key_state_front_ = is_down; break;
    case GLFW_KEY_S:
    case GLFW_KEY_DOWN: key_state_back_ = is_down; break;
    case GLFW_KEY_A:
    case GLFW_KEY_LEFT: key_state_left_ = is_down; break;
    case GLFW_KEY_D:
    case GLFW_KEY_RIGHT: key_state_right_ = is_down; break;
    case GLFW_KEY_SPACE: key_state_up_ = is_down; break;
    case GLFW_KEY_LEFT_SHIFT: key_state_down_ = is_down; break;
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
    case 1: // right
    {
      if(is_down)
      {
        mouse_drag_start_position_ = mouse_current_position_;
      }

      is_dragging_mouse_ = is_down;
      break;
    }
    case 2: // middle
    {
      if(is_down)
      {
        mouse_drag_start_position_ = mouse_current_position_;
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
    mouse_current_position_ = {x, y};

    if(mouse_btn_states_[1])
    {
      const auto delta = (mouse_current_position_ - mouse_drag_start_position_) * mouse_rotation_sensitivity_;
      mouse_drag_start_position_ = mouse_current_position_;

      view_angles_.x += delta.x; // yaw
      view_angles_.y += delta.y; // pitch

      if(view_angles_.y > 89.0f)
        view_angles_.y = 89.0f;
      if(view_angles_.y < -89.0f)
        view_angles_.y = -89.0f;

      recomputeDirectionVector();
    }
    else if(mouse_btn_states_[2])
    {
      const auto delta = (mouse_current_position_ - mouse_drag_start_position_) * (mouse_translate_sensitivity_ * 0.5f);
      mouse_drag_start_position_ = mouse_current_position_;

      world_eye_position_ -= world_eye_right_ * delta.x;
      world_eye_position_ += world_eye_up_ * delta.y;

      recomputeDirectionVector();
    }
  }

  void Camera::processUserMouseScroll(float delta_time, float xoffset, float yoffset)
  {
    (void)delta_time;
    (void)xoffset;
    world_eye_position_ += world_eye_front_ * (yoffset * mouse_scroll_sensitivity_);
  }

  void Camera::update()
  {
    if(key_state_front_)
      world_eye_position_ += world_eye_front_ * 0.1f;
    if(key_state_back_)
      world_eye_position_ -= world_eye_front_ * 0.1f;
    if(key_state_right_)
      world_eye_position_ += world_eye_right_ * 0.1f;
    if(key_state_left_)
      world_eye_position_ -= world_eye_right_ * 0.1f;
    if(key_state_up_)
      world_eye_position_ += world_eye_up_ * 0.1f;
    if(key_state_down_)
      world_eye_position_ -= world_eye_up_ * 0.1f;
  }

} // namespace owds