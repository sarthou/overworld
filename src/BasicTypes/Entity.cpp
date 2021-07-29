#include "overworld/BasicTypes/Entity.h"

#include <ros/ros.h>

namespace owds {
    
Entity::Entity(const std::string& id): id_(id), is_located_(false), bullet_id_(-1)
{}

void Entity::updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation)
{
    updatePose(translation, rotation, ros::Time::now());
}

void Entity::updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation, ros::Time stamp)
{
    pose_ = Pose(translation, rotation);
    is_located_ = true;
    last_pose_ = stamp;
}

const owds::Pose& Entity::pose() const
{
    if (!is_located_){
        throw UnlocatedEntityError(id_);
    }
    return pose_;
}

} // namespace owds
