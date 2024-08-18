#include "overworld/Engine/Graphics/GLFW/CameraUpdater.h"

#include <GLFW/glfw3.h>
#include <array>
#include <cstdint>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "overworld/Engine/Graphics/Common/Camera.h"
#include "overworld/Engine/Graphics/Common/CameraProjection.h"
#include "overworld/Engine/Graphics/Common/CameraView.h"
#include "overworld/Engine/Graphics/Common/ViewAntiAliasing.h"
#include "overworld/Utils/GlmMath.h"

namespace owds {

  CameraUpdater::CameraUpdater(Camera* camera)
  {
    if(camera == nullptr)
      camera_ = new Camera;
    else
      camera_ = new Camera(*camera);
  }

  void CameraUpdater::initCamera(Camera* camera)
  {
    delete camera_;
    camera_ = new Camera(*camera);
  }

  void CameraUpdater::processUserKeyboardInput(const float delta_time, const int key, const bool is_down)
  {
    if(camera_ == nullptr)
      return;

    switch(key)
    {
    case GLFW_KEY_F2: camera_->render_collision_models_ = is_down; break;
    case GLFW_KEY_F3:
    {
      if(is_down == false && key_state_debug_)
        camera_->render_debug_ = !camera_->render_debug_;
      key_state_debug_ = is_down;
      break;
    }
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

  void CameraUpdater::processUserMouseBtnInput(const float delta_time, const char btn, const bool is_down)
  {
    if(camera_ == nullptr)
      return;

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

  void CameraUpdater::processUserMouseInput(const float delta_time, const float x, const float y)
  {
    if(camera_ == nullptr)
      return;

    (void)delta_time;
    mouse_current_position_ = {x, y};

    if(mouse_btn_states_[1])
    {
      const auto delta = (mouse_current_position_ - mouse_drag_start_position_) * mouse_rotation_sensitivity_;
      mouse_drag_start_position_ = mouse_current_position_;

      camera_->view_angles_.x += delta.x; // yaw
      camera_->view_angles_.y += delta.y; // pitch

      if(camera_->view_angles_.y > 89.0f)
        camera_->view_angles_.y = 89.0f;
      if(camera_->view_angles_.y < -89.0f)
        camera_->view_angles_.y = -89.0f;

      camera_->recomputeDirectionVector();
    }
    else if(mouse_btn_states_[2])
    {
      const auto delta = (mouse_current_position_ - mouse_drag_start_position_) * (mouse_translate_sensitivity_ * 0.5f);
      mouse_drag_start_position_ = mouse_current_position_;

      camera_->world_eye_position_ -= camera_->world_eye_right_ * delta.x;
      camera_->world_eye_position_ += camera_->world_eye_up_ * delta.y;

      camera_->recomputeDirectionVector();
    }
  }

  void CameraUpdater::processUserMouseScroll(float delta_time, float xoffset, float yoffset)
  {
    if(camera_ == nullptr)
      return;

    (void)delta_time;
    (void)xoffset;
    camera_->world_eye_position_ += camera_->world_eye_front_ * (yoffset * mouse_scroll_sensitivity_);
  }

  void CameraUpdater::update()
  {
    if(camera_ == nullptr)
      return;

    if(key_state_front_)
      camera_->world_eye_position_ += camera_->world_eye_front_ * 0.1f;
    if(key_state_back_)
      camera_->world_eye_position_ -= camera_->world_eye_front_ * 0.1f;
    if(key_state_right_)
      camera_->world_eye_position_ += camera_->world_eye_right_ * 0.1f;
    if(key_state_left_)
      camera_->world_eye_position_ -= camera_->world_eye_right_ * 0.1f;
    if(key_state_up_)
      camera_->world_eye_position_ += camera_->world_eye_up_ * 0.1f;
    if(key_state_down_)
      camera_->world_eye_position_ -= camera_->world_eye_up_ * 0.1f;
  }

  void CameraUpdater::setCameraView(const owds::CameraView_e view_type)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->view_type_ = view_type;
  }

  void CameraUpdater::setProjection(const owds::CameraProjection_e proj)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->projection_type_ = proj;
  }

  void CameraUpdater::setFieldOfView(const float fov)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->field_of_view_ = glm::radians(fov);
  }

  void CameraUpdater::setOutputAA(const owds::ViewAntiAliasing_e aa_setting)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->aa_setting_ = aa_setting;
  }

  void CameraUpdater::setOutputFPS(const std::uint64_t desired_target_fps)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    (void)desired_target_fps; // todo
  }

  void CameraUpdater::setOutputResolution(const std::array<float, 2>& resolution)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->view_dimensions_[0] = resolution[0];
    camera_->view_dimensions_[1] = resolution[1];
    camera_->has_changed_size_ = true;
  }

  void CameraUpdater::setPlanes(const std::array<float, 2>& near_far_planes)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->planes_[0] = near_far_planes[0];
    camera_->planes_[1] = near_far_planes[1];
    camera_->updateProjectionMatrix();
  }

  void CameraUpdater::setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    // to verify
    camera_->world_eye_position_ = ToV3(position);
    camera_->view_angles_.x = orientation[2]; // yaw
    camera_->view_angles_.y = orientation[1]; // pitch
    camera_->recomputeDirectionVector();
  }

  void CameraUpdater::setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->world_eye_position_ = ToV3(eye_position);
    camera_->world_eye_front_ = ToV3(dst_position) - camera_->world_eye_position_;

    const auto xy = glm::sqrt(glm::pow(camera_->world_eye_front_.x, 2) + glm::pow(camera_->world_eye_front_.y, 2));

    camera_->view_angles_.x = glm::degrees(glm::atan(camera_->world_eye_front_.y, camera_->world_eye_front_.x));
    camera_->view_angles_.y = glm::degrees(glm::atan(camera_->world_eye_front_.z, static_cast<float>(xy)));

    camera_->recomputeDirectionVector();
    camera_->updateViewMatrix();
  }

  void CameraUpdater::setPositionAndDirection(const std::array<float, 3>& eye_position, const std::array<float, 3>& eye_direction)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->world_eye_position_ = ToV3(eye_position);
    camera_->world_eye_front_ = ToV3(eye_direction);

    const auto xy = glm::sqrt(glm::pow(camera_->world_eye_front_.x, 2) + glm::pow(camera_->world_eye_front_.y, 2));

    camera_->view_angles_.x = glm::degrees(glm::atan(camera_->world_eye_front_.y, camera_->world_eye_front_.x));
    camera_->view_angles_.y = glm::degrees(glm::atan(camera_->world_eye_front_.z, static_cast<float>(xy)));

    camera_->recomputeDirectionVector();
    camera_->updateViewMatrix();
  }

  void CameraUpdater::setDirectionAndLookAt(const std::array<float, 3>& eye_direction, const std::array<float, 3>& dst_position)
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->world_eye_front_ = ToV3(eye_direction);
    camera_->world_eye_position_ = ToV3(dst_position) - camera_->world_eye_front_;

    const auto xy = glm::sqrt(glm::pow(camera_->world_eye_front_.x, 2) + glm::pow(camera_->world_eye_front_.y, 2));

    camera_->view_angles_.x = glm::degrees(glm::atan(camera_->world_eye_front_.y, camera_->world_eye_front_.x));
    camera_->view_angles_.y = glm::degrees(glm::atan(camera_->world_eye_front_.z, static_cast<float>(xy)));

    camera_->recomputeDirectionVector();
    camera_->updateViewMatrix();
  }

  void CameraUpdater::finalize()
  {
    assert(camera_ && "CameraUpdater work on null pointer");
    camera_->recomputeDirectionVector();
    camera_->updateMatrices();
  }

} // namespace owds