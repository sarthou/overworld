#include "overworld/BasicTypes/Entity.h"

#include <ros/ros.h>

namespace owds{
    
Entity::Entity(): id_(""), isLocated_(false){

}
Entity::Entity(const std::string& id): id_(id), isLocated_(false){

}

void Entity::updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation){
    updatePose(translation, rotation, ros::Time::now());
}

void Entity::updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation, ros::Time stamp){
    pose_ = Pose(translation, rotation);
    isLocated_ = true;
    lastPose_ = stamp;
}

void Entity::unsetPose() {
    isLocated_ = false;
}
bool Entity::isLocated() const{
    return isLocated_;
}

const owds::Pose& Entity::pose() const{
    if (!isLocated_){
        throw UnlocatedEntityError(id_);
    }
    return pose_;
}

void Entity::setId(const std::string& id){
    id_ = id;
}

const std::string& Entity::id() const{
    return id_;
}
}
