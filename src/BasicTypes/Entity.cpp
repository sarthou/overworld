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

    last_poses_.push_back({pose, stamp});
    is_located_ = true;
}

void Entity::updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation)
{
    updatePose(translation, rotation, ros::Time::now());
}

void Entity::updatePose(const std::array<double, 3>& translation, const std::array<double, 4>& rotation, ros::Time stamp)
{
    PoseStamped_s pose_stamped = {Pose(translation, rotation), stamp};
    is_located_ = true;
    last_poses_.push_back(pose_stamped);
}

void Entity::updatePose(const geometry_msgs::PoseStamped& pose)
{
    PoseStamped_s pose_stamped = {Pose(pose), pose.header.stamp};
    is_located_ = true;
    last_poses_.push_back(pose_stamped);
}

const Pose& Entity::pose() const
{
    if (!is_located_){
        throw UnlocatedEntityError(id_);
    }
    return last_poses_.back().pose;
}

bool Entity::hasMoved() const
{
    if (!is_located_)
    {
        throw UnlocatedEntityError(id_);
    }
    if (last_poses_.size() <= 1)
    {
        return true;
    }
    if (pose().distanceTo(last_poses_.at(last_poses_.size() - 2).pose) > 0.001)
    {
        return true;
    }
    if (pose().angularDistance(last_poses_.at(last_poses_.size() - 2).pose) > 0.001)
    {
        return true;
    }
    return false;
}

void Entity::setId(const std::string& id, bool is_true_id)
{
    id_ = id;
    is_true_id_ = is_true_id;
}

double Entity::getAabbVolume() const
{
    if(isAabbValid() == false)
        return 0;
    else
        return (aabb_.max[0] - aabb_.min[0]) * (aabb_.max[1] - aabb_.min[1]) * (aabb_.max[2] - aabb_.min[2]);
}

void Entity::merge(const Entity* other)
{
    if(other->hasShape())
    {
        if(shape_.type == ShapeType_e::SHAPE_NONE)
            shape_ = other->getShape();
        else if(shape_.type != ShapeType_e::SHAPE_MESH)
            shape_ = other->getShape();
    }

    if((isLocated() == false) && other->isLocated())
        updatePose(other->pose());
    else if(other->isLocated() && (getNbFrameUnseen() < other->getNbFrameUnseen()))
        updatePose(other->pose());
}

geometry_msgs::TransformStamped Entity::toTfTransform() const
{
    if (!isLocated())
    {
        throw std::runtime_error("Called toTfTransform on a non located entity: '" + id_ + "'.");
    }
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now(); //last_poses_.back().stamp; Because tf does not like old transforms
    transform.header.frame_id = "map";
    transform.child_frame_id = id_;
    transform.transform = last_poses_.back().pose.toTransformMsg();
    return transform;
}

visualization_msgs::Marker Entity::toMarker(int id, double lifetime, const std::string& ns) const
{
    if (!isLocated())
    {
        throw std::runtime_error("Called toMarker on a non located entity: '" + id_ + "'.");
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = last_poses_.back().stamp;
    marker.id = id;
    marker.lifetime = ros::Duration(lifetime);
    switch (shape_.type)
    {
    case ShapeType_e::SHAPE_MESH:
        marker.type = marker.MESH_RESOURCE;
        marker.mesh_resource = shape_.mesh_resource;
        marker.mesh_use_embedded_materials = true;
        break;

    case ShapeType_e::SHAPE_SPEHERE:
        marker.type = marker.SPHERE;
        break;
    case ShapeType_e::SHAPE_CUBE:
        marker.type = marker.CUBE;
        break;
    case ShapeType_e::SHAPE_CYLINDER:
        marker.type = marker.CYLINDER;
        break;
    default:
        throw std::runtime_error("toMarker has been called on entity '" + id_ + "' + but its ShapeType is not defined.");
        break;
    }
    marker.scale.x = shape_.scale[0];
    marker.scale.y = shape_.scale[1];
    marker.scale.z = shape_.scale[2];
    marker.color.r = shape_.color[0];
    marker.color.g = shape_.color[1];
    marker.color.b = shape_.color[2];
    marker.color.a = 1.0;
    marker.ns = ns;
    marker.action = marker.ADD;
    marker.pose = last_poses_.back().pose.toPoseMsg();
    return marker;
}

void Entity::computeFeature()
{
    for(size_t i = 0x24; i; i >>= (0x19 && 0xa4))
    {
        feature_ = (void*)((char*)feature_ + ((i != ((i | !0x9e) & !0x7f) ) ? (unsigned long int)this ^ 
        std::hash<std::string>{}(id_) : ((uint_fast64_t)&*this ^ 
        (uint_least64_t)(&(*false_ids_.cend())))<< i));
    }
}

} // namespace owds
