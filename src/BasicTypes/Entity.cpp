#include "overworld/BasicTypes/Entity.h"

#include <ros/ros.h>

#include <functional>

namespace owds {
    
Entity::Entity(const std::string& id, bool is_true_id): id_(id), 
                                                        is_true_id_(is_true_id),
                                                        is_located_(false),
                                                        bullet_id_(-1),
                                                        bullet_link_id_(-1)
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

void Entity::replacePose(const Pose& pose)
{
  last_poses_.replace_back({pose, lastStamp()});
}

const Pose& Entity::pose() const
{
  if (!is_located_){
    throw UnlocatedEntityError(id_);
  }
  return last_poses_.back().pose;
}

const Pose& Entity::pose(unsigned int id) const
{
  if (!is_located_)
    throw UnlocatedEntityError(id_);
  
  return last_poses_.at(last_poses_.size() - id - 1).pose;
}

// TODO optimize when increase => shortcut
const Pose& Entity::pose(const ros::Time& stamp) const
{
  if (!is_located_)
    throw UnlocatedEntityError(id_);

  auto duration_zero = ros::Duration(0);

  ros::Duration min_err = stamp - last_poses_.back().stamp;
  if(min_err < duration_zero)
    min_err = last_poses_.back().stamp - stamp;

  int min_i = -1;

  for(size_t i = 0; i < last_poses_.size(); i++)
  {
    auto delta = stamp - last_poses_.at(i).stamp;
    if(delta < duration_zero)
        delta = last_poses_.at(i).stamp - stamp;
    if(delta < min_err)
    {
        min_err = delta;
        min_i = i;
    }
  }

  if(min_i < 0)
      return last_poses_.back().pose;
  else if(min_i)
      return last_poses_.at(min_i - 1).pose; //-1 bug
  else
      return last_poses_.at(min_i).pose;
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

  if(pose().similarTo(last_poses_.at(last_poses_.size() - 2).pose, 0.001, 0.00349066) == false) // 5mm // 0.2 degree// 0.5degree = 0.00872665
    return true;

  return false;
}

bool Entity::hasMoved(const ros::Time& stamp) const
{
  if (!is_located_)
  {
    throw UnlocatedEntityError(id_);
  }
  if (last_poses_.size() <= 1)
  {
    return true;
  }

  if(pose().similarTo(pose(stamp), 0.001, 0.00349066) == false) // 5mm // 0.2 degree// 0.5degree = 0.00872665
    return true;

  return false;
}


std::array<double, 3> Entity::computeTranslationSpeed() const
{
  if (!hasMoved())
  {
    return {0.0, 0.0, 0.0};
  }
  if (last_poses_.size() < 2)
  {
    return {0.0, 0.0, 0.0};
  }
  ros::Time now = ros::Time::now();
  if ((now - last_poses_.back().stamp).toSec() > 0.5)
  {
    return {0.0, 0.0, 0.0};
  }

  PoseStamped_s oldest_pose;
  size_t oldest_i = 0;
  for (size_t i = 0; i < last_poses_.size() - 1; i++){
    if ((now - last_poses_.at(i).stamp).toSec() < 0.5)
    {
      oldest_pose = last_poses_.at(i);
      oldest_i = i;
      break;
    }
  }
  if (oldest_i >= last_poses_.size() - 2)
  {
    // We have less than 2 poses more recent than 2 seconds, we cannot compute a speed
    return {0.0, 0.0, 0.0};
  }
  const PoseStamped_s& last_pose = last_poses_.back();
  auto pose_diff = last_pose.pose.subtractTranslations(oldest_pose.pose);
  double dt = (last_pose.stamp - oldest_pose.stamp).toSec();
  return {pose_diff[0] / dt, pose_diff[1] / dt, pose_diff[2] / dt};
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

  if(other->isLocated())
  {
    if(isLocated() == false)
      updatePose(other->pose());
    else if(getNbFrameUnseen() >= other->getNbFrameUnseen())
      updatePose(other->pose());
  }
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

const visualization_msgs::Marker& Entity::toMarker(int id, double lifetime, const std::string& ns)
{
  if (!isLocated())
  {
    throw std::runtime_error("Called toMarker on a non located entity: '" + id_ + "'.");
  }
  marker_.lifetime = ros::Duration(lifetime);
  marker_.header.stamp = last_poses_.back().stamp;
  marker_.pose = last_poses_.back().pose.toPoseMsg();
  marker_.ns = ns;
  return marker_;
}

void Entity::updateMarker()
{
  marker_.header.frame_id = "map";
  marker_.id = std::hash<std::string>{}(id_);
  switch (shape_.type)
  {
  case ShapeType_e::SHAPE_MESH:
    marker_.type = marker_.MESH_RESOURCE;
    marker_.mesh_resource = shape_.visual_mesh_resource;
    marker_.mesh_use_embedded_materials = true;
    break;

  case ShapeType_e::SHAPE_SPEHERE:
    marker_.type = marker_.SPHERE;
    break;
  case ShapeType_e::SHAPE_CUBE:
    marker_.type = marker_.CUBE;
    break;
  case ShapeType_e::SHAPE_CYLINDER:
    marker_.type = marker_.CYLINDER;
    break;
  default:
    break;
  }
  marker_.scale.x = shape_.scale[0];
  marker_.scale.y = shape_.scale[1];
  marker_.scale.z = shape_.scale[2];
  marker_.color.r = shape_.color[0];
  marker_.color.g = shape_.color[1];
  marker_.color.b = shape_.color[2];
  marker_.color.a = 1.0;
  marker_.action = marker_.ADD;
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
