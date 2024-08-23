#ifndef OWDS_VIRTUALCAMERA_H
#define OWDS_VIRTUALCAMERA_H

#include <array>
#include <cstdint>

#include "overworld/Engine/Common/Camera/CameraUpdater.h"
#include "overworld/Engine/Common/Camera/CameraView.h"

namespace owds {

  class VirtualCamera
  {
  public:
    VirtualCamera(unsigned int width, unsigned int height, float fov, owds::CameraView_e view_type, float near_plane, float far_plane);
    ~VirtualCamera();

    Camera* getCamera() { return camera_.getCamera(); }

    void setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position);
    void setPositionAndDirection(const std::array<float, 3>& eye_position, const std::array<float, 3>& eye_direction);

    uint32_t** getImageData() { return &image_; }
    uint8_t* getImage() { return (uint8_t*)image_; }

    unsigned int getWidth() const { return width_; }
    unsigned int getHeight() const { return height_; }

    void requestUpdate()
    {
      need_update_ = true;
      updated_ = false;
    }
    void setUpdated()
    {
      updated_ = true;
      need_update_ = false;
    }
    void setImageUsed() { updated_ = false; }

    bool isUpdated() const { return updated_; }
    bool doNeedUpdate() const { return need_update_; }

  private:
    CameraUpdater camera_;
    uint32_t* image_;

    unsigned int width_;
    unsigned int height_;

    bool need_update_;
    bool updated_;
  };

} // namespace owds

#endif // OWDS_VIRTUALCAMERA_H