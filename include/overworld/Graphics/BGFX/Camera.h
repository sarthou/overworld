#ifndef OWDS_GRAPHICS_BGFX_CAMERA_H
#define OWDS_GRAPHICS_BGFX_CAMERA_H

#include <array>
#include <bitset>
#include <cstdint>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Graphics/Base/Camera.h"

namespace owds {
  class World;
}

namespace owds::bgfx {
  class Camera final : public owds::Camera
  {
  private:
    ::bgfx::ViewId id_{};
    owds::CameraView_e view_type_{};
    owds::CameraProjection_e projection_type_{};
    std::array<float, 2> view_dimensions_{};
    float field_of_view_ = 0.785398; // 45 deg
    glm::vec3 world_eye_position_{};
    glm::vec3 world_eye_front_{};
    glm::vec3 world_eye_right_{};
    glm::vec3 world_eye_up_ = {0.f, 0., 1.f};
    std::array<float, 16> view_matrix_;
    std::array<float, 16> proj_matrix_;

    glm::vec2 view_angles_{-90.0f, 0.0f}; // yaw/pitch in deg

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

    Camera() = default;
    void setViewId(::bgfx::ViewId id) { id_ = id; }

    void updateViewMatrix();
    void updateProjectionMatrix();
    void updateMatrices();

    void setCameraView(owds::CameraView_e view_type) override;
    void setProjection(owds::CameraProjection_e proj) override;
    void setFieldOfView(float fov) override;
    void setOutputAA(owds::ViewAntiAliasing_e aa_setting) override;
    void setOutputFPS(std::uint64_t desired_target_fps) override;
    void setOutputResolution(const std::array<float, 2>& resolution) override;
    void setOutputClearColor(owds::Color clear_color) override;
    void setCaptureCallback(std::function<void(const std::vector<owds::Color>&)> callback) override;
    void finalize() override;
    void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation) override;
    void setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position) override;

    void recomputeDirectionVector();

    void processUserKeyboardInput(float delta_time, int key, bool is_down) override;
    void processUserMouseBtnInput(float delta_time, char btn, bool is_down) override;
    void processUserMouseInput(float delta_time, float x, float y) override;
    void processUserMouseScroll(float delta_time, float xoffset, float yoffset) override;
    void update() override;

    glm::vec3 getPosition() const { return world_eye_position_; }

  private:
    // glm::mat3 to_z_up_;
  };
} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_CAMERA_H
