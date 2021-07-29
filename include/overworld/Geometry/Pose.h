#ifndef OWDS_POSE_H
#define OWDS_POSE_H

#include <array>

#include <eigen3/Eigen/Geometry>

namespace owds{

class Pose
{
public:
    Pose();
    Pose(const Pose& pose);

    /**
     * @brief Construct a new Pose object
     * 
     * @param translation translation part of the pose (in meters)
     * @param rotation rotation part of the pose. Quaternion (x, y, z, w)
     */
    Pose(const std::array<double,3>& translation, const std::array<double, 4>& rotation);

    double distanceSqTo(const Pose& pose) const;
    double distanceTo(const Pose& pose) const;

    const std::pair<std::array<double, 3>, std::array<double, 4>> arrays() const;

protected:
    Eigen::Affine3d t_;
};

} // namespace owds

#endif // OWDS_POSE_H
