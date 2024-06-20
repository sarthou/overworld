#ifndef OWDS_GRAPHICS_BGFX_CAMERA_H
#define OWDS_GRAPHICS_BGFX_CAMERA_H

#include <array>
#include <cstdint>

#include "overworld/Graphics/BGFX/API.h"
#include "overworld/Graphics/Base/Camera.h"

namespace owds {
  class World;
}

namespace owds::bgfx {
  class Camera final : public owds::Camera
  {
  public:
    ::bgfx::ViewId id_{};
    owds::CameraView_e view_type_{};
    owds::CameraProjection_e projection_type_{};
    std::array<float, 2> view_dimensions_{};
    float aspect_ratio_{};
    float field_of_view_ = 1.f;
    std::array<float, 3> world_eye_position_{};
    std::array<float, 3> world_eye_front_{};
    std::array<float, 3> world_eye_up_ = {0.f, 0., 1.f};
    std::array<float, 16> view_matrix_;
    std::array<float, 16> proj_matrix_;
    std::reference_wrapper<owds::World> currently_viewed_world_;

    explicit Camera(owds::World& world);

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
  };
} // namespace owds::bgfx

#endif // OWDS_GRAPHICS_BGFX_CAMERA_H
