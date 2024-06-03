#ifndef OWDS_GEOMETRYUTILS_H
#define OWDS_GEOMETRYUTILS_H

#include <overworld/Geometry/Pose.h>

namespace owds {

  inline double getCameraYawFromHeadPose(const Pose& head_pose)
  {
    // We assume the head is pointing in the positive z axis (camera convention, true for robot head)
    Pose head_z_axis = head_pose * Pose({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0});
    // We take the angle between x axis of map frame and the projection of the z head axis on the xy plane of the map
    return atan2(head_z_axis.getY() - head_pose.getY(), head_z_axis.getX() - head_pose.getX());
  }

} // namespace owds

#endif /* OWDS_GEOMETRYUTILS_H */
