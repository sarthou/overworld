#ifndef POSE_H
#define POSE_H
#include <eigen3/Eigen/Geometry>

class Pose{
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

    protected:
    Eigen::Affine3d t_;
};
#endif /* POSE_H */
