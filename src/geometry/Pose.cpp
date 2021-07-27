#include "overworld/geometry/Pose.h"

Pose::Pose(): t_(Eigen::Affine3d::Identity()){

}

Pose::Pose(const Pose& pose): t_(pose.t_){

}

Pose::Pose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation): t_(Eigen::Translation3d(
                translation[0], translation[1], translation[2]) * Eigen::Quaternion<double>(rotation[3], rotation[0], rotation[1], rotation[2])
                ){

}

double Pose::distanceSqTo(const Pose& pose) const{
    return (t_.translation() - pose.t_.translation()).squaredNorm();
}

double Pose::distanceTo(const Pose& pose) const{
    return (t_.translation() - pose.t_.translation()).norm();
}