#ifndef OWDS_GRAPHICS_BASE_CAMERA_H
#define OWDS_GRAPHICS_BASE_CAMERA_H

#include <functional>

#include "overworld/Graphics/Base/CameraProjection.h"
#include "overworld/Graphics/Base/CameraView.h"
#include "overworld/Graphics/Base/Color.h"
#include "overworld/Graphics/Base/ViewAntiAliasing.h"

namespace owds {
  class Camera
  {
  public:
    virtual ~Camera() noexcept = default;

    /**
     * @param view_type What each option does is outlined where owds::CameraViewType is declared.
     */
    virtual void setCameraView(owds::CameraView_e view_type) = 0;

    /**
     * @param proj What each option does is outlined where owds::CameraProjection is declared.
     */
    virtual void setProjection(owds::CameraProjection_e proj) = 0;

    /**
     * Note: This only works when perspective projection is used.
     *
     * @param fov Self-explanatory
     */
    virtual void setFieldOfView(float fov) = 0;

    /**
     * @param mode todo
     */
    virtual void setOutputAA(owds::ViewAntiAliasing_e mode) = 0;

    /**
     * @param desired_target_fps If this value is zero, vsync is assumed to be desired.
     */
    virtual void setOutputFPS(std::uint64_t desired_target_fps) = 0;

    /**
     * Note: This setting may be ignored if the camera is currently previewed in a window.
     * @param resolution Sets the framebuffer resolution for the camera's view, in pixels.
     */
    virtual void setOutputResolution(const std::array<float, 2>& resolution) = 0;

    /**
     * @param clear_color 8-bit RGBA color
     */
    virtual void setOutputClearColor(owds::Color clear_color) = 0;

    /**
     * You may set a capture callback which is called every frame containing the framebuffer's pixel data.
     */
    virtual void setCaptureCallback(std::function<void(const std::vector<owds::Color>&)> callback) = 0;

    /**
     * Initializes some internal data structures and some settings won't be modifiable anymore after this call.
     */
    virtual void finalize() = 0;

    /**
     * @param position Position.
     * @param orientation XYZ rotations (Yaw + Pitch + Roll), in radians.
     */
    virtual void setPositionAndOrientation(const std::array<float, 3>& position, const std::array<float, 3>& orientation) = 0;

    /**
     * @param eye_position Camera position.
     * @param dst_position The position where to look at.
     */
    virtual void setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position) = 0;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_CAMERA_H
