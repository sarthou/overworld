#ifndef OWDS_AREA_H
#define OWDS_AREA_H

#include <string>
#include <unordered_set>

#include "overworld/Geometry/Pose.h"
#include "overworld/Geometry/Polygon.h"
#include "overworld/BasicTypes/Entity.h"

namespace owds {

class Area
{
public:
  Area(const std::string& id, const Pose& center, double radius, double half_height);
  Area(const std::string& id, const Polygon& polygon, double z_min, double height);

  void setHysteresis(double hysteresis);
  void setOwner(Entity* owner) { owner_ = owner; }

  bool isStatic() { return (owner_ != nullptr); }
  bool isEmpty();

  bool isInside(Entity* entity);
  bool setOut(Entity* entity);

private:
  std::string id_;
  Pose center_;
  Entity* owner_;
  bool is_circle_;

  double radius_;
  double half_height_;

  float z_min_;
  float z_max_;
  Polygon polygon_;

  double hysteresis_distance_;
  std::unordered_set<Entity*> upcoming_entities_;
  std::unordered_set<Entity*> inside_entities_;
  std::unordered_set<Entity*> leaving_entities_;

  bool isInCircle(Entity* entity);
  bool isInPolygon(Entity* entity);

  bool entityInLeaving(Entity* entity);
  bool entityInUpcoming(Entity* entity);
  bool entityInArea(Entity* entity);
};

} // namespace owds

#endif // OWDS_AREA_H