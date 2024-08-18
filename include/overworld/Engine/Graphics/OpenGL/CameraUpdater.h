#ifndef OWDS_CAMEAUPDATER_H
#define OWDS_CAMEAUPDATER_H

#include <array>
#include <bitset>
#include <cstdint>
#include <glm/vec2.hpp>

#include "overworld/Engine/Graphics/Common/CameraProjection.h"
#include "overworld/Engine/Graphics/Common/CameraView.h"
#include "overworld/Engine/Graphics/Common/ViewAntiAliasing.h"
#include "overworld/Engine/Graphics/OpenGL/Camera.h"
#include "overworld/Helper/GlmMath.h"

namespace owds {

  class CameraUpdater
  {
  public:
    CameraUpdater(Camera* camera = nullptr);
    ~CameraUpdater() { delete camera_; }

    void initCamera(Camera* camera);
    Camera* getCamera() const { return camera_; }

    void processUserKeyboardInput(float delta_time, int key, bool is_down);
    void processUserMouseBtnInput(float delta_time, char btn, bool is_down);
    void processUserMouseInput(float delta_time, float x, float y);
    void processUserMouseScroll(float delta_time, float xoffset, float yoffset);
    void update();

    void setCameraView(owds::CameraView_e view_type);
    void setProjection(owds::CameraProjection_e proj);
    void setFieldOfView(float fov);
    void setOutputAA(owds::ViewAntiAliasing_e aa_setting);
    void setOutputFPS(std::uint64_t desired_target_fps);
    void setOutputResolution(const std::array<float, 2>& resolution);
    void setPlanes(const std::array<float, 2>& near_far_planes);

    void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation);
    void setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position);
    void setPositionAndDirection(const std::array<float, 3>& eye_position, const std::array<float, 3>& eye_direction);
    void setDirectionAndLookAt(const std::array<float, 3>& eye_direction, const std::array<float, 3>& dst_position);

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

    std::bitset<16> mouse_btn_states_{};

    float mouse_rotation_sensitivity_ = 0.1;
    float mouse_translate_sensitivity_ = 0.035;
    float mouse_scroll_sensitivity_ = 0.2;
  };
} // namespace owds

#endif // OWDS_CAMEAUPDATER_H