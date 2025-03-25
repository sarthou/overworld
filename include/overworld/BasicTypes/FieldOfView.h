#ifndef OWDS_FIELDOFVIEW_H
#define OWDS_FIELDOFVIEW_H

#include "overworld/Geometry/Pose.h"

namespace owds {

#define TO_HALF_RAD M_PI / 180. / 2.

  class FieldOfView
  {
  public:
    FieldOfView(double height,
                double width,
                double clip_near,
                double clip_far) : height_(height),
                                   width_(width),
                                   clip_near_(clip_near),
                                   clip_far_(clip_far),
                                   opengl_ratio_(std::tan(width_ * TO_HALF_RAD) / std::tan(height_ * TO_HALF_RAD))
    {}

    double getHeight() const { return height_; }
    double getWidth() const { return width_; }
    double getClipNear() const { return clip_near_; }
    double getClipFar() const { return clip_far_; }

    double getRatio() const { return width_ / height_; }
    double getRatioOpenGl() const { return opengl_ratio_; }

    /**
     * @brief
     *
     * @param pose the pose in the right frame of reference (the FOV origin frame, camera convention)
     * @return true
     * @return false
     */
    bool hasIn(const Pose& pose, double margin = 0.) const
    {
      return pose.getZ() <= clip_far_ && std::abs(pose.getOriginTilt()) <= (height_ - margin * 2) * TO_HALF_RAD &&
             std::abs(pose.getOriginPan()) <= (width_ - margin * 2) * TO_HALF_RAD;
    }

    std::string toString() const
    {
      return " - Height: " + std::to_string(height_) +
             "\n - Width: " + std::to_string(width_) +
             "\n - Clip near: " + std::to_string(clip_near_) +
             "\n - Clip far: " + std::to_string(clip_far_);
    }

  private:
    double height_;    // degrees
    double width_;     // degrees
    double clip_near_; // meters
    double clip_far_;  // meters
    double opengl_ratio_;
  };

} // namespace owds

#endif // OWDS_FIELDOFVIEW_H