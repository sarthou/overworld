#include "overworld/BasicTypes/Area.h"

namespace owds {

Area::Area(const std::string& id, const Pose& center, double radius, double half_height) : id_(id),
                                                                                           center_(center),
                                                                                           is_circle_(true),
                                                                                           polygon_({}),
                                                                                           radius_(radius),
                                                                                           half_height_(half_height),
                                                                                           hysteresis_distance_(0.0001)

{}

Area::Area(const std::string& id, const Pose& center, const Polygon& polygon,
                                  double z_min, double height) : id_(id),
                                                                 center_(center),
                                                                 is_circle_(false),
                                                                 polygon_(polygon),
                                                                 z_min_(z_min),
                                                                 z_max_(z_min + height),
                                                                 hysteresis_distance_(0.0001)
{}

void Area::setHysteresis(double hysteresis)
{
  hysteresis_distance_ = hysteresis + 0.0001;
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

bool Area::isInCircle(Entity* entity)
{
  Pose entity_pose = entity->pose();
  Pose circle_pose = center_;

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

  if((entity_pose.getZ() > z_max_) ||
     (entity_pose.getZ() < z_min_))
  {
    setOut(entity);
    return false;
  }

  //create the inner and outer segments for each segments of the base polygon
  // /!\ At this point we can say if segments are really inside or ouside the base polygon
  std::vector<segement_t> inner_segments = polygon_.offsetingPolygon(hysteresis_distance_);
	std::vector<segement_t> outer_segments = polygon_.offsetingPolygon(-hysteresis_distance_);

  // The vectrice extraction will detect witch is inside and witch is outside the base polygon
	std::vector<point_t> enter_poly;
	std::vector<point_t> leave_poly;
	polygon_.extractVectrices(outer_segments, inner_segments, enter_poly, leave_poly);

  point_t entity_point(entity_pose.getX(), entity_pose.getY());

  if(polygon_.isInside(entity_point, leave_poly) == false)
  {
    setOut(entity);
    return false;
  }
  else if(polygon_.isInside(entity_point) == false)
    return entityInLeaving(entity);
  else if(polygon_.isInside(entity_point, enter_poly) == true)
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