#include "overworld/BasicTypes/Entity.h"

#include <ros/ros.h>

namespace owds {
    
Entity::Entity(const std::string& id, bool is_true_id): id_(id), 
                                                        is_true_id_(is_true_id),
                                                        is_located_(false),
                                                        bullet_id_(-1)
{}

void Entity::updatePose(const Pose& pose, ros::Time stamp)
{
    pose_ = pose;
    is_located_ = true;
    last_pose_ = stamp;
}

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

void Entity::updatePose(const geometry_msgs::PoseStamped& pose)
{
    pose_ = Pose(pose);
    is_located_ = true;
    last_pose_ = pose.header.stamp;
}

const Pose& Entity::pose() const
{
    if (!is_located_){
        throw UnlocatedEntityError(id_);
    }
    return pose_;
}

void Entity::setId(const std::string& id, bool is_true_id)
{
    id_ = id;
    is_true_id_ = is_true_id;
}

} // namespace owds
