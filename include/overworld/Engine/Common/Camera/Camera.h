#ifndef OWDS_CAMERA_H
#define OWDS_CAMERA_H

#include <array>
#include <cstdint>
#include <functional>
#include <glm/matrix.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <vector>

#include "overworld/Engine/Common/Camera/CameraProjection.h"
#include "overworld/Engine/Common/Camera/CameraView.h"
#include "overworld/Engine/Common/Camera/ViewAntiAliasing.h"

namespace owds {

  class CameraUpdater;

  class Camera
  {
    friend CameraUpdater;

  private:
    owds::CameraView_e view_type_{};
    owds::CameraProjection_e projection_type_{};
    owds::ViewAntiAliasing_e aa_setting_;

    float field_of_view_ = 0.785398; // 45 deg
    glm::vec3 world_eye_position_{};
    glm::vec3 world_eye_front_{};
    glm::vec3 world_eye_right_{};
    glm::vec3 world_eye_up_ = {0.f, 0., 1.f};

    glm::vec2 view_angles_{-90.0f, 0.0f}; // yaw/pitch in deg
    glm::vec2 planes_{0.1, 100.};

    bool render_debug_ = true;
    bool render_collision_models_ = false;

    std::array<float, 2> view_dimensions_{};
    bool has_changed_size_ = false;

  public:
    Camera() = default;

    glm::vec3 getPosition() const { return world_eye_position_; }
    glm::vec3 getFrontPose() const { return world_eye_position_ + world_eye_front_; }

    glm::mat4 getViewMatrix() const { return view_matrix_; }
    glm::mat4 getProjectionMatrix() const { return proj_matrix_; }
    ViewAntiAliasing_e getAASetting() const { return aa_setting_; }

    float getNearPlane() const { return planes_[0]; }
    float getFarPlane() const { return planes_[1]; }

    float getWidth() const { return view_dimensions_.at(0); }
    float getHeight() const { return view_dimensions_.at(1); }
    bool sizeHasChanged() const { return has_changed_size_; }

    bool shouldRenderDebug() const { return render_debug_; }
    bool shouldRendercollisionModels() const { return render_collision_models_; }

    std::vector<glm::vec4> getFrustumCornersWorldSpace();

  private:
    glm::mat4 view_matrix_;
    glm::mat4 proj_matrix_;

    void updateViewMatrix();
    void updateProjectionMatrix();
    void updateMatrices();

    void recomputeDirectionVector();
  };
} // namespace owds

#endif // OWDS_CAMERA_H
