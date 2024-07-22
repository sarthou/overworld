#ifndef OWDS_GRAPHICS_OPENGL_CAMERA_H
#define OWDS_GRAPHICS_OPENGL_CAMERA_H

#include <array>
#include <bitset>
#include <cstdint>
#include <functional>
#include <glm/matrix.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <vector>

#include "overworld/Engine/Graphics/Common/CameraProjection.h"
#include "overworld/Engine/Graphics/Common/CameraView.h"
#include "overworld/Engine/Graphics/Common/ViewAntiAliasing.h"

namespace owds {

  class Camera
  {
  private:
    unsigned long id_{};
    owds::CameraView_e view_type_{};
    owds::CameraProjection_e projection_type_{};
    float field_of_view_ = 0.785398; // 45 deg
    glm::vec3 world_eye_position_{};
    glm::vec3 world_eye_front_{};
    glm::vec3 world_eye_right_{};
    glm::vec3 world_eye_up_ = {0.f, 0., 1.f};

    glm::vec2 view_angles_{-90.0f, 0.0f}; // yaw/pitch in deg
    glm::vec2 planes_{0.1, 100.};

    glm::vec2 mouse_current_position_{};
    glm::vec2 mouse_drag_start_position_{};
    bool is_dragging_mouse_ = false;

    bool key_state_front_ = false;
    bool key_state_back_ = false;
    bool key_state_left_ = false;
    bool key_state_right_ = false;
    bool key_state_up_ = false;
    bool key_state_down_ = false;

    std::bitset<16> mouse_btn_states_{};

    float mouse_rotation_sensitivity_ = 0.1;
    float mouse_translate_sensitivity_ = 0.035;
    float mouse_scroll_sensitivity_ = 0.2;

  public:
    bool show_debug_stats_ = false;
    bool render_collision_models_ = false;
    owds::ViewAntiAliasing_e aa_setting_;
    std::array<float, 2> view_dimensions_{};

    Camera() = default;
    void setViewId(unsigned long id) { id_ = id; }

    void updateViewMatrix();
    void updateProjectionMatrix();
    void updateMatrices();

    void setCameraView(owds::CameraView_e view_type);
    void setProjection(owds::CameraProjection_e proj);
    void setFieldOfView(float fov);
    void setOutputAA(owds::ViewAntiAliasing_e aa_setting);
    void setOutputFPS(std::uint64_t desired_target_fps);
    void setOutputResolution(const std::array<float, 2>& resolution);
    void setPlanes(const std::array<float, 2>& near_far_planes);
    void finalize();
    void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation);
    void setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position);

    void recomputeDirectionVector();

    void processUserKeyboardInput(float delta_time, int key, bool is_down);
    void processUserMouseBtnInput(float delta_time, char btn, bool is_down);
    void processUserMouseInput(float delta_time, float x, float y);
    void processUserMouseScroll(float delta_time, float xoffset, float yoffset);
    void update();

    glm::vec3 getPosition() const { return world_eye_position_; }

    glm::mat4 getViewMatrix() const { return view_matrix_; }
    glm::mat4 getProjectionMatrix() const { return proj_matrix_; }

  private:
    glm::mat4 view_matrix_;
    glm::mat4 proj_matrix_;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_OPENGL_CAMERA_H
