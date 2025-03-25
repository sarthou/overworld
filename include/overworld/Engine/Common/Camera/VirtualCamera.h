#ifndef OWDS_VIRTUALCAMERA_H
#define OWDS_VIRTUALCAMERA_H

#include <array>
#include <cstdint>
#include <unordered_set>

#include "overworld/Engine/Common/Camera/CameraUpdater.h"
#include "overworld/Engine/Common/Camera/CameraView.h"

namespace owds {

  class VirtualCamera
  {
  public:
    VirtualCamera(unsigned int width, unsigned int height, float fov /*Rad*/, owds::CameraView_e view_type, float near_plane, float far_plane);
    ~VirtualCamera();
    VirtualCamera(const VirtualCamera& other) = delete;
    VirtualCamera& operator=(const VirtualCamera&) = delete;

    VirtualCamera(VirtualCamera&& other) : camera_(std::move(other.camera_)),
                                           image_(std::move(other.image_)),
                                           width_(other.width_),
                                           height_(other.height_),
                                           need_update_(other.need_update_),
                                           updated_(other.updated_)
    {
      other.image_ = nullptr;
    }

    Camera* getCamera() { return camera_.getCamera(); }

    void setPositionAndLookAt(const std::array<double, 3>& eye_position, const std::array<double, 3>& dst_position);
    void setPositionAndDirection(const std::array<double, 3>& eye_position, const std::array<double, 3>& eye_direction);
    void setPositionAndOrientation(const std::array<double, 3>& eye_position, const std::array<double, 4>& orientation);

    uint32_t** getImageData() { return &image_; }
    uint8_t* getImage() { return (uint8_t*)image_; }
    std::unordered_set<uint32_t> getSegmentedIds() const;

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