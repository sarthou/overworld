#ifndef OWDS_AREA_H
#define OWDS_AREA_H

#include <string>
#include <unordered_set>

#include "overworld/BasicTypes/Entity.h"
#include "overworld/Geometry/Polygon.h"
#include "overworld/Geometry/Pose.h"

namespace owds {

  class Area
  {
  public:
    Area(const std::string& id, const Pose& center, double radius, double half_height);
    Area(const std::string& id, const Polygon& polygon, double z_min, double height);

    const std::string& id() const { return id_; }

    const std::unordered_set<int>& getWorldIds() const { return engine_ids_; }
    void setWorldIds(const std::unordered_set<int>& engine_ids) { engine_ids_ = engine_ids; }

    void setHysteresis(double hysteresis);
    void setOwner(Entity* owner) { owner_ = owner; }
    void setOwnerStr(const std::string& owner_str) { owner_str_ = owner_str; }

    bool isStatic() const { return (owner_str_ == ""); }
    bool isEmpty() const;
    bool isCircle() const { return is_circle_; }

    Entity* getOwner() const { return owner_; }
    const std::string& getOwnerStr() const { return owner_str_; }
    const Polygon& getPolygon() const { return polygon_; }
    double getZmin() const { return z_min_; }
    double getZmax() const { return z_max_; }
    double getRadius() const { return radius_; }
    double getHalfHeight() const { return half_height_; }
    const Pose& getCenterPose() const { return center_; }

    bool isInside(const Pose& pose, bool hysteresis = true);
    bool isInside(Entity* entity);
    bool setOut(Entity* entity);
    void clearInsideEntities();

  private:
    std::string id_;
    Pose center_;
    Entity* owner_;
    std::string owner_str_;
    bool is_circle_;

    double radius_;
    double half_height_;

    double z_min_;
    double z_max_;
    Polygon polygon_;

    double hysteresis_distance_;
    std::unordered_set<Entity*> upcoming_entities_;
    std::unordered_set<Entity*> inside_entities_;
    std::unordered_set<Entity*> leaving_entities_;

    std::unordered_set<int> engine_ids_;

    bool isInCircle(Entity* entity);
    bool isInPolygon(Entity* entity);

    bool entityInLeaving(Entity* entity);
    bool entityInUpcoming(Entity* entity);
    bool entityInArea(Entity* entity);
  };

} // namespace owds

#endif // OWDS_AREA_H