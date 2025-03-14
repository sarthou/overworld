#ifndef OWDS_CAMEAUPDATER_H
#define OWDS_CAMEAUPDATER_H

#include <array>
#include <bitset>
#include <cstdint>
#include <glm/vec2.hpp>

#include "overworld/Engine/Common/Camera/Camera.h"
#include "overworld/Engine/Common/Camera/CameraProjection.h"
#include "overworld/Engine/Common/Camera/CameraView.h"
#include "overworld/Engine/Common/Camera/ViewAntiAliasing.h"
#include "overworld/Utils/GlmMath.h"

namespace owds {

  enum Key_e
  {
    key_unknown,
    key_esc,
    key_enter,
    key_f1,
    key_f2,
    key_f3,
    key_f4,
    key_f5,
    key_f6,
    key_f7,
    key_f8,
    key_f9,
    key_f10,
    key_f11,
    key_f12,
    key_0,
    key_1,
    key_2,
    key_3,
    key_4,
    key_5,
    key_6,
    key_7,
    key_8,
    key_9,
    key_up,
    key_down,
    key_left,
    key_right,
    key_space,
    key_minus,
    key_equal,
    key_a,
    key_b,
    key_c,
    key_d,
    key_e,
    key_f,
    key_g,
    key_h,
    key_i,
    key_j,
    key_k,
    key_l,
    key_m,
    key_n,
    key_o,
    key_p,
    key_q,
    key_r,
    key_s,
    key_t,
    key_u,
    key_v,
    key_w,
    key_x,
    key_y,
    key_z,
    key_left_shift
  };

  class CameraUpdater
  {
  public:
    CameraUpdater(Camera* camera = nullptr);
    ~CameraUpdater() { delete camera_; }
    CameraUpdater(const CameraUpdater& other) = delete;
    CameraUpdater& operator = (const CameraUpdater&) = delete;

    CameraUpdater(CameraUpdater&& other)
    {
      camera_ = std::move(other.camera_);
      other.camera_ = nullptr;
    }

    Camera* getCamera() const { return camera_; }

    void processUserKeyboardInput(float delta_time, Key_e key, bool is_down);
    void processUserMouseBtnInput(float delta_time, char btn, bool is_down);
    void processUserMouseInput(float delta_time, float x, float y);
    void processUserMouseScroll(float delta_time, float xoffset, float yoffset);
    void update();

    void setCameraView(owds::CameraView_e view_type);
    void setProjection(owds::CameraProjection_e proj);
    void setFieldOfView(float fov);
    void setFieldOfViewRad(float fov);
    void setOutputAA(owds::ViewAntiAliasing_e aa_setting);
    void setOutputFPS(std::uint64_t desired_target_fps);
    void setOutputResolution(const std::array<float, 2>& resolution);
    void setPlanes(const std::array<float, 2>& near_far_planes);

    void setPositionAnd2DOrientation(const std::array<double, 3>& position, const std::array<double, 3>& orientation);
    void setPositionAndLookAt(const std::array<double, 3>& eye_position, const std::array<double, 3>& dst_position);
    void setPositionAndDirection(const std::array<double, 3>& eye_position, const std::array<double, 3>& eye_direction);
    void setDirectionAndLookAt(const std::array<double, 3>& eye_direction, const std::array<double, 3>& dst_position);
    void setPositionAndOrientation(const std::array<double, 3>& eye_position, const std::array<double, 4>& orientation);

    void clearChanges() { camera_->has_changed_size_ = false; }

    void finalize();

  private:
    Camera* camera_ = nullptr;

    glm::vec2 mouse_current_position_{};
    glm::vec2 mouse_drag_start_position_{};
    bool is_dragging_mouse_ = false;

    bool key_state_front_ = false;
    bool key_state_back_ = false;
    bool key_state_left_ = false;
    bool key_state_right_ = false;
    bool key_state_up_ = false;
    bool key_state_down_ = false;
    bool key_state_debug_ = false;
    bool key_state_shadows_ = false;

    std::bitset<16> mouse_btn_states_{};

    float mouse_rotation_sensitivity_ = 0.1;
    float mouse_translate_sensitivity_ = 0.035;
    float mouse_scroll_sensitivity_ = 0.2;
  };
} // namespace owds

#endif // OWDS_CAMEAUPDATER_H