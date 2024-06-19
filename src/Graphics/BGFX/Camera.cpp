#include "overworld/Graphics/BGFX/Camera.h"

#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Helper/GlmMath.h"

namespace owds::bgfx {
  Camera::Camera(owds::World& world)
    : currently_viewed_world_(world) {}

  void Camera::updateViewMatrix()
  {
    view_matrix_ = FromM4(glm::lookAt(
      ToV3(world_eye_position_),
      ToV3(world_eye_position_) + ToV3(world_eye_front_),
      ToV3(world_eye_up_)));

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
  }

  void Camera::setOutputFPS(const std::uint64_t desired_target_fps)
  {
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
  }

  void Camera::finalize()
  {
    updateViewMatrix();
  }

  void Camera::setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation)
  {
  }

  void Camera::setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position)
  {
    world_eye_position_ = eye_position;
    world_eye_front_ = FromV3(ToV3(dst_position) - ToV3(eye_position));
    updateViewMatrix();
  }
} // namespace owds::bgfx