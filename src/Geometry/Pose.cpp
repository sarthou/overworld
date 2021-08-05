#include "overworld/Geometry/Pose.h"

namespace owds{

Pose::Pose(): t_(Eigen::Affine3d::Identity())
{}

Pose::Pose(const Pose& pose): t_(pose.t_){}

Pose::Pose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation): t_(Eigen::Translation3d(
                translation[0], translation[1], translation[2]) * Eigen::Quaternion<double>(rotation[3], rotation[0], rotation[1], rotation[2])
                )
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
    Eigen::Quaternion<double> rot(t_.rotation());
    return rot.angularDistance(Eigen::Quaternion<double>(pose.t_.rotation()));
}

std::pair<std::array<double, 3>, std::array<double, 4>> Pose::arrays() const
{
    Eigen::Vector3d translation(t_.translation());
    Eigen::Quaternion<double> rot(t_.rotation());
    std::pair<std::array<double, 3>, std::array<double, 4>> p;
    std::array<double, 3> t();
    p.first = {translation.x(), translation.y(), translation.z()};
    p.second = {rot.x(), rot.y(), rot.z(), rot.w()};
    return p;
}

double Pose::getOriginTilt() const
{
    Eigen::Vector3d origin = t_.translation();
    return std::acos(origin.z() / std::hypot(origin.y(), origin.z()));
}

double Pose::getOriginPan() const
{
    Eigen::Vector3d origin = t_.translation();
    return std::acos(origin.z() / std::hypot(origin.x(), origin.z()));
}

Pose Pose::transformIn(const Pose& new_frame) const 
{
    return new_frame.t_.inverse() * t_;
}

Pose Pose::operator*(const Pose& b) const
{
    Pose p;
    p.t_ = this->t_ * b.t_;
    return p;
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
    return t_.rotation().eulerAngles(0,1,2)[0];
}

double Pose::getPitch() const
{
    return t_.rotation().eulerAngles(0,1,2)[1];
}

double Pose::getYaw() const
{
    return t_.rotation().eulerAngles(0,1,2)[2];
}

} // namespace owds