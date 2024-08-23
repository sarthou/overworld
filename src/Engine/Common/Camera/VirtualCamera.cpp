#include "overworld/Engine/Common/Camera/VirtualCamera.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <unordered_set>

#include "overworld/Engine/Common/Camera/CameraProjection.h"
#include "overworld/Engine/Common/Camera/CameraView.h"

namespace owds {

  VirtualCamera::VirtualCamera(unsigned int width, unsigned int height,
                               float fov, owds::CameraView_e view_type,
                               float near_plane, float far_plane) : width_(width),
                                                                    height_(height)
  {
    camera_.setCameraView(view_type);
    camera_.setProjection(CameraProjection_e::perspective);
    camera_.setFieldOfView(fov);
    camera_.setOutputResolution({(float)width, (float)height});
    camera_.setPlanes({near_plane, far_plane});
    camera_.finalize();

    image_ = new uint32_t[width_ * height_];
  }

  VirtualCamera::~VirtualCamera()
  {
    delete[] image_;
  }

  void VirtualCamera::setPositionAndLookAt(const std::array<float, 3>& eye_position, const std::array<float, 3>& dst_position)
  {
    camera_.setPositionAndLookAt(eye_position, dst_position);
  }

  void VirtualCamera::setPositionAndDirection(const std::array<float, 3>& eye_position, const std::array<float, 3>& eye_direction)
  {
    camera_.setPositionAndDirection(eye_position, eye_direction);
  }

  std::unordered_set<uint32_t> VirtualCamera::getSegmentedIds() const
  {
    std::cout << "[VirtualCamera] getSegmentedIds" << std::endl;
    uint32_t n = width_ * height_;
    std::unordered_set<uint32_t> res(image_, image_ + n);
    res.erase(std::numeric_limits<uint32_t>::max());
    return res;
  }

} // namespace owds