#ifndef OWDS_POSE_H
#define OWDS_POSE_H

#include <array>
#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace owds{

class Pose
{
public:
    Pose();
    Pose(const Pose& pose);
    explicit Pose(const Eigen::Affine3d& pose) : t_(pose) {}

    /**
     * @brief Construct a new Pose object
     * 
     * @param translation translation part of the pose (in meters)
     * @param rotation rotation part of the pose. Quaternion (x, y, z, w)
     */
    Pose(const std::array<double,3>& translation, const std::array<double, 4>& rotation);

    /**
     * @brief Construct a new Pose object
     * 
     * @param translation translation part of the pose (in meters)
     * @param rotation_euler rotation in euler angles (radians) as defined by the 3-2-1 convention (yaw, pitch, roll)
     */
    Pose(const std::array<double, 3>& translation, const std::array<double, 3>& rotation_euler);

    explicit Pose(const geometry_msgs::TransformStamped& transform);
    explicit Pose(const geometry_msgs::PoseStamped& pose);

    bool operator==(const Pose& other) const { return t_.isApprox(other.t_); }
    bool operator!=(const Pose& other) const { return !t_.isApprox(other.t_); }

    double distanceSqTo(const Pose& pose) const;
    double distanceTo(const Pose& pose) const;

    /**
     * @brief Returns the difference in angle between two poses.
     * It does NOT return the angle from one pose to the origin of the other one.
     * 
     * @param pose 
     * @return double 
     */
    double angularDistance(const Pose& pose) const;

    std::array<double, 3> subtractTranslations(const Pose& other) const;

    std::pair<std::array<double, 3>, std::array<double, 4>> arrays() const;
    double getOriginTilt() const;
    double getOriginPan() const;

    bool similarTo(const Pose& other, double translation_delta = 0.01, double angular_delta = 0.0872665) const; // default values are 1 cm and 5 degrees

    /**
     * @brief Compute the transform of this pose in "new_frame"
     * 
     * @param new_frame 
     * @return Pose 
     */
    Pose transformIn(const Pose& new_frame) const;
    Pose operator* (const Pose& b) const;

    geometry_msgs::Transform toTransformMsg() const;
    geometry_msgs::Pose toPoseMsg() const;

    double getX() const;
    double getY() const;
    double getZ() const;

    double getRoll() const;
    double getPitch() const;
    double getYaw() const;

    Pose lerpTo(const Pose& goal, double alpha) const;

    friend std::ostream& operator<<(std::ostream& os, const Pose& pose);

protected:
    Eigen::Affine3d t_;
};

} // namespace owds

#endif // OWDS_POSE_H
