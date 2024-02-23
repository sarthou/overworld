#include "overworld/BasicTypes/Area.h"

namespace owds {

Area::Area(const std::string& id, const Pose& center, double radius, double half_height) : id_(id),
                                                                                           center_(center),
                                                                                           owner_(nullptr),
                                                                                           is_circle_(true),
                                                                                           polygon_({}),
                                                                                           radius_(radius),
                                                                                           half_height_(half_height),
                                                                                           hysteresis_distance_(0.0001)

{}

Area::Area(const std::string& id, const Polygon& polygon,
                                  double z_min, double height) : id_(id),
                                                                 owner_(nullptr),
                                                                 is_circle_(false),
                                                                 polygon_(polygon),
                                                                 z_min_(z_min),
                                                                 z_max_(z_min + height),
                                                                 hysteresis_distance_(0.0001)
{}

void Area::setHysteresis(double hysteresis)
{
  hysteresis_distance_ = hysteresis + 0.0001;
  if(is_circle_ == false)
    polygon_.setHysteresis(hysteresis_distance_);
}

bool Area::isEmpty() const
{
  size_t nb_entity = leaving_entities_.size() +
                     inside_entities_.size();

  return (nb_entity == 0);
}

bool Area::isInside(const Pose& pose, bool hysteresis)
{
  if(hysteresis)
  {
    Entity entity_wrapper("plop");
    entity_wrapper.updatePose(pose);
    return isInside(&entity_wrapper);
  }
  else
  {
    if(is_circle_)
    {
      double entity_distance = pose.distanceTo(center_);
      return (entity_distance <= radius_);
    }
    else
    {
      point_t entity_point(pose.getX(), pose.getY());
      return polygon_.isInside(entity_point);
    }
  }
}

bool Area::isInside(Entity* entity)
{
  if(is_circle_)
    return isInCircle(entity);
  else
    return isInPolygon(entity);
}

bool Area::setOut(Entity* entity)
{
  size_t nb = 0;
  upcoming_entities_.erase(entity);
  nb += inside_entities_.erase(entity);
  nb += leaving_entities_.erase(entity);

  return (nb != 0);
}

void Area::clearInsideEntities()
{
  upcoming_entities_.clear();
  inside_entities_.clear();
  leaving_entities_.clear();
}

bool Area::isInCircle(Entity* entity)
{
  Pose entity_pose = entity->pose();
  Pose circle_pose = center_;
  if(owner_ != nullptr)
  {
    if(owner_->isLocated() == false)
    {
      setOut(entity);
      return false;
    }
    circle_pose = owner_->pose() * center_;
  }

  double up_limit = circle_pose.getZ() + half_height_;
  double down_limit = circle_pose.getZ() - half_height_;

  if((entity_pose.getZ() > up_limit) ||
     (entity_pose.getZ() < down_limit))
  {
    setOut(entity);
    return false;
  }

  double entity_distance = entity_pose.distanceTo(circle_pose);
  double leave_radius = radius_ + hysteresis_distance_;
  double enter_radius = radius_ - hysteresis_distance_;

  if(entity_distance > leave_radius)
  {
    setOut(entity);
    return false;
  }
  else if(entity_distance > radius_)
    return entityInLeaving(entity);
  else if(entity_distance > enter_radius)
    return entityInUpcoming(entity);
  else
    return entityInArea(entity);
}

bool Area::isInPolygon(Entity* entity)
{
  Pose entity_pose = entity->pose();
  Pose polygon_pose = center_;
  if(owner_ != nullptr)
  {
    if(owner_->isLocated() == false)
    {
      setOut(entity);
      return false;
    }
    polygon_pose = owner_->pose();
    polygon_.transformIn(owner_->pose());
  }

  double up_limit = polygon_pose.getZ() + z_max_;
  double down_limit = polygon_pose.getZ() + z_min_;

  if((entity_pose.getZ() > up_limit) ||
     (entity_pose.getZ() < down_limit))
  {
    setOut(entity);
    return false;
  }

  point_t entity_point(entity_pose.getX(), entity_pose.getY());

  if(polygon_.isInsideOuter(entity_point) == false)
  {
    setOut(entity);
    return false;
  }
  else if(polygon_.isInside(entity_point) == false)
    return entityInLeaving(entity);
  else if(polygon_.isInsideInner(entity_point) == false)
    return entityInUpcoming(entity);
  else
    return entityInArea(entity);
}

bool Area::entityInLeaving(Entity* entity)
{
  if(inside_entities_.erase(entity) != 0)
  {
    leaving_entities_.insert(entity);
    return true;
  }
  else if(leaving_entities_.find(entity) != leaving_entities_.end())
    return true;
  else
  {
    upcoming_entities_.erase(entity);
    return false;
  }
}

bool Area::entityInUpcoming(Entity* entity)
{
  if(inside_entities_.find(entity) != inside_entities_.end())
    return true;
  else if(leaving_entities_.erase(entity) != 0)
  {
    inside_entities_.insert(entity);
    return true;
  }
  else
  {
    upcoming_entities_.insert(entity);
    return false;
  }
}

bool Area::entityInArea(Entity* entity)
{
  leaving_entities_.erase(entity);
  upcoming_entities_.erase(entity);
  inside_entities_.insert(entity);
  return true;
}

} // namespace owds