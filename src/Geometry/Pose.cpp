#include "overworld/Geometry/Pose.h"

namespace owds{

Pose::Pose(): t_(Eigen::Affine3d::Identity())
{}

Pose::Pose(const Pose& pose): t_(pose.t_){}

Pose::Pose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation): t_(Eigen::Translation3d(
                translation[0], translation[1], translation[2]) * Eigen::Quaternion<double>(rotation[3], rotation[0], rotation[1], rotation[2])
                )
{}

double Pose::distanceSqTo(const Pose& pose) const
{
    return (t_.translation() - pose.t_.translation()).squaredNorm();
}

double Pose::distanceTo(const Pose& pose) const
{
    return (t_.translation() - pose.t_.translation()).norm();
}

double Pose::angleTo(const Pose& pose) const
{
    Eigen::Quaternion<double> rot(t_.rotation());
    return rot.angularDistance(Eigen::Quaternion<double>(pose.t_.rotation()));
}

const std::pair<std::array<double, 3>, std::array<double, 4>> Pose::arrays() const
{
    Eigen::Vector3d translation(t_.translation());
    Eigen::Quaternion<double> rot(t_.rotation());
    std::pair<std::array<double, 3>, std::array<double, 4>> p;
    std::array<double, 3> t();
    p.first = {translation.x(), translation.y(), translation.z()};
    p.second = {rot.x(), rot.y(), rot.z(), rot.w()};
    return p;
}

Pose Pose::transform(const Pose& new_frame) const 
{
    return new_frame.t_.inverse() * t_;
}

double getX();
    double getY();
    double getZ();

} // namespace owds