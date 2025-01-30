#include "overworld/Geometry/Pose.h"

#include <array>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>

namespace owds {

  Pose::Pose() : t_(Eigen::Affine3d::Identity())
  {}

  Pose::Pose(const Pose& pose) : t_(pose.t_) {}

  Pose::Pose(const Eigen::Affine3d& pose) : t_(pose) {}

  Pose::Pose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation) : t_(Eigen::Translation3d(
                                                                                                     translation[0], translation[1], translation[2]) *
                                                                                                   Eigen::Quaternion<double>(rotation[3], rotation[0], rotation[1], rotation[2]))
  {}

  Pose::Pose(const std::array<double, 3>& translation, const std::array<double, 3>& rotation_euler)
    : t_(Eigen::Translation3d(translation[0], translation[1], translation[2]) * Eigen::AngleAxisd(rotation_euler[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rotation_euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rotation_euler[0], Eigen::Vector3d::UnitX()))
  {}

  Pose::Pose(const geometry_msgs::TransformStamped& transform)
  {
    Eigen::Translation3d tra(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    Eigen::Quaternion<double> qua(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
    t_ = Eigen::Affine3d(tra * qua);
  }

  Pose::Pose(const geometry_msgs::PoseStamped& pose)
  {
    Eigen::Translation3d tra(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    Eigen::Quaternion<double> qua(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    t_ = Eigen::Affine3d(tra * qua);
  }

  bool Pose::operator==(const Pose& other) const
  {
    return t_.isApprox(other.t_);
  }

  bool Pose::operator!=(const Pose& other) const
  {
    return !t_.isApprox(other.t_);
  }

  double Pose::distanceSqTo(const Pose& pose) const
  {
    return (t_.translation() - pose.t_.translation()).squaredNorm();
  }

  double Pose::distanceTo(const Pose& pose) const
  {
    return (t_.translation() - pose.t_.translation()).norm();
  }

  double Pose::angularDistance(const Pose& pose) const
  {
    Eigen::Quaternion<double> rot(t_.linear());
    return rot.angularDistance(Eigen::Quaternion<double>(pose.t_.linear()));
  }

  std::array<double, 3> Pose::subtractTranslations(const Pose& other) const
  {
    Eigen::Vector3d diff(t_.translation() - other.t_.translation());
    return {diff.x(), diff.y(), diff.z()};
  }

  std::pair<std::array<double, 3>, std::array<double, 4>> Pose::arrays() const
  {
    const Eigen::Vector3d& tra = t_.translation();
    Eigen::Quaterniond quat(t_.linear());
    return {
      {tra.x(), tra.y(), tra.z()},
      {quat.x(), quat.y(), quat.z(), quat.w()}
    };
  }

  double Pose::getOriginTilt() const
  {
    const Eigen::Vector3d& origin = t_.translation();
    return std::acos(origin.z() / std::hypot(origin.y(), origin.z()));
  }

  double Pose::getOriginPan() const
  {
    const Eigen::Vector3d& origin = t_.translation();
    return std::acos(origin.z() / std::hypot(origin.x(), origin.z()));
  }

  bool Pose::similarTo(const Pose& other, double translation_delta, double angular_delta) const
  {
    if(distanceTo(other) > translation_delta)
      return false;
    else if(angularDistance(other) > angular_delta)
      return false;
    else
      return true;
  }

  Pose Pose::transformIn(const Pose& new_frame) const
  {
    return Pose(new_frame.t_.inverse() * t_);
  }

  Pose Pose::operator*(const Pose& b) const
  {
    return Pose(this->t_ * b.t_);
  }

  geometry_msgs::Transform Pose::toTransformMsg() const
  {
    geometry_msgs::Transform transform;
    const Eigen::Vector3d& tra = t_.translation();
    transform.translation.x = tra.x();
    transform.translation.y = tra.y();
    transform.translation.z = tra.z();

    // Extract rotation matrix and compute quaternion in-place
    Eigen::Quaterniond quat(t_.linear());  // Use `linear()` instead of `rotation()`
    
    transform.rotation.x = quat.x();
    transform.rotation.y = quat.y();
    transform.rotation.z = quat.z();
    transform.rotation.w = quat.w();
    return transform;
  }

  geometry_msgs::Pose Pose::toPoseMsg() const
  {
    geometry_msgs::Pose pose;
    const Eigen::Vector3d& tra(t_.translation());
    pose.position.x = tra.x();
    pose.position.y = tra.y();
    pose.position.z = tra.z();

    // Extract rotation matrix and compute quaternion in-place
    Eigen::Quaterniond quat(t_.linear());  // Use `linear()` instead of `rotation()`

    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return pose;
  }

  bool Pose::isTranslationZero() const
  {
    return t_.translation().isMuchSmallerThan(0.01);
  }

  double Pose::getX() const
  {
    return t_.translation().x();
  }

  double Pose::getY() const
  {
    return t_.translation().y();
  }

  double Pose::getZ() const
  {
    return t_.translation().z();
  }

  double Pose::getRoll() const
  {
    return t_.linear().eulerAngles(2, 1, 0)[0];
  }

  double Pose::getPitch() const
  {
    return t_.linear().eulerAngles(2, 1, 0)[1];
  }

  double Pose::getYaw() const
  {
    return t_.linear().eulerAngles(2, 1, 0)[2];
  }

  Pose Pose::lerpTo(const Pose& goal, double alpha) const
  {
    Eigen::Quaternion<double> rot1(t_.linear());
    Eigen::Quaternion<double> rot2(goal.t_.linear());

    Eigen::Vector3d trans1 = t_.translation();
    Eigen::Vector3d trans2 = goal.t_.translation();

    Eigen::Affine3d result;
    result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
    result.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();

    return Pose(result);
  }

  std::ostream& operator<<(std::ostream& os, const Pose& pose)
  {
    os << pose.getX() << " : " << pose.getY() << " : " << pose.getZ() << " --  " << pose.getRoll() << " : " << pose.getPitch() << " : " << pose.getYaw();
    return os;
  }

} // namespace owds