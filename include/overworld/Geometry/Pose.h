#ifndef OWDS_POSE_H
#define OWDS_POSE_H

#include <array>

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace owds{

class Pose
{
public:
    Pose();
    Pose(const Pose& pose);
    Pose(const Eigen::Affine3d& pose) { t_ = pose; }

    /**
     * @brief Construct a new Pose object
     * 
     * @param translation translation part of the pose (in meters)
     * @param rotation rotation part of the pose. Quaternion (x, y, z, w)
     */
    Pose(const std::array<double,3>& translation, const std::array<double, 4>& rotation);

    Pose(const geometry_msgs::TransformStamped& transform);
    Pose(const geometry_msgs::PoseStamped& pose);

    double distanceSqTo(const Pose& pose) const;
    double distanceTo(const Pose& pose) const;

    /**
     * @brief Returns the difference in angle between two poses. It does NOT return the angle from one pose to the origin of the other one.
     * 
     * @param pose 
     * @return double 
     */
    double angularDistance(const Pose& pose) const;

    std::pair<std::array<double, 3>, std::array<double, 4>> arrays() const;
    double getOriginTilt() const;
    double getOriginPan() const;

    /**
     * @brief Compute the transform of this pose in "new_frame"
     * 
     * @param new_frame 
     * @return Pose 
     */
    Pose transformIn(const Pose& new_frame) const;
    Pose& operator*= (const Pose& b);

    double getX() const;
    double getY() const;
    double getZ() const;

    double getRoll() const;
    double getPitch() const;
    double getYaw() const;

protected:
    Eigen::Affine3d t_;
};

inline Pose operator*(Pose a, const Pose& b){
    a *= b;
    return a;
}

} // namespace owds

#endif // OWDS_POSE_H
