#include "overworld/Perception/Managers/AreasPerceptionManager.h"

#define CIRCLE_STEPS 10

namespace owds {

AreasPerceptionManager::~AreasPerceptionManager()
{
  for(auto& area : areas_)
    delete area.second;
  areas_.clear();
}

bool AreasPerceptionManager::update()
{
  if((pending_percepts_.size() == 0) && !this->shouldRun())
    return false;

  for(const auto& module : this->perception_modules_)
    if(module.second->isActivated() && module.second->hasBeenUpdated())
      module.second->accessPercepts([this](std::map<std::string, Area>& percepts){ this->getPercepts(percepts); });

  solvePendingAreas();

  return true;
}

void AreasPerceptionManager::getPercepts(std::map<std::string, Area>& percepts)
{
  for(auto& percept : percepts)
  {
    auto it = areas_.find(percept.second.id());
    if(it == areas_.end())
    {
      Area* new_entity = new Area(percept.second);
      if(percept.second.isStatic() == true)
      {
        it = areas_.insert(std::pair<std::string, Area*>(percept.second.id(), new_entity)).first;
        addToBullet(it->second);
      }
      else
        pending_percepts_.insert(std::pair<std::string, Area*>(percept.first, new_entity));
    }
  }
}

void AreasPerceptionManager::solvePendingAreas()
{
  std::unordered_set<std::string> solved;
  for(auto percept : pending_percepts_)
  {
    Entity* owner = findAreaOwner(percept.second);
    if(owner != nullptr)
    {
      percept.second->setOwner(owner);
      auto it = areas_.insert(std::pair<std::string, Area*>(percept.second->id(), percept.second)).first;
      addToBullet(it->second);
      solved.insert(it->first);
    }
  }

  for(auto id : solved)
    pending_percepts_.erase(id);
}

void AreasPerceptionManager::addToBullet(Area* area)
{
  if(area->isCircle())
    addCircleToBullet(area);
  else
    addPolygonToBullet(area);
}

void AreasPerceptionManager::addPolygonToBullet(Area* area)
{
  int owner_id = -1;
  if(area->isStatic() == false)
    owner_id = area->getOwner()->bulletId();

  std::unordered_set<int> bullet_ids;
  auto polygon_points = area->getPolygon().getBasePoints();
  double z_min = area->getZmin();
  double z_max = area->getZmax();
  for(size_t i = 0, j = 1; i < polygon_points.size(); i++, j++)
  {
    if(j >= polygon_points.size())
      j = 0;

    bullet_ids.insert(bullet_client_->addUserDebugLine({polygon_points[i].x, polygon_points[i].y, z_min},
                                                       {polygon_points[i].x, polygon_points[i].y, z_max},
                                                       {1,0,0}, 2, 0, -1, owner_id));
    bullet_ids.insert(bullet_client_->addUserDebugLine({polygon_points[i].x, polygon_points[i].y, z_min},
                                                       {polygon_points[j].x, polygon_points[j].y, z_min},
                                                       {1,0,0}, 2, 0, -1, owner_id));
    bullet_ids.insert(bullet_client_->addUserDebugLine({polygon_points[i].x, polygon_points[i].y, z_max},
                                                       {polygon_points[j].x, polygon_points[j].y, z_max},
                                                       {1,0,0}, 2, 0, -1, owner_id));
  }

  area->setBulletIds(bullet_ids);
}

void AreasPerceptionManager::addCircleToBullet(Area* area)
{
  int owner_id = -1;
  if(area->isStatic() == false)
    owner_id = area->getOwner()->bulletId();

  std::unordered_set<int> bullet_ids;

  double angle_step = (2 * M_PI / CIRCLE_STEPS);
  double radius = area->getRadius();
  double z_min = area->getCenterPose().getZ() - area->getHalfHeight();
  double z_max = area->getCenterPose().getZ() + area->getHalfHeight();
  double x_center = area->getCenterPose().getX();
  double y_center = area->getCenterPose().getY();
  for(size_t i = 0; i < CIRCLE_STEPS; i++)
  {
    double angle = i * angle_step;
    double x = x_center + radius * std::cos(angle);
    double y = y_center + radius * std::sin(angle);
    double angle_next = (i+1) * angle_step;
    double x_next = x_center + radius * std::cos(angle_next);
    double y_next = y_center + radius * std::sin(angle_next);
    bullet_ids.insert(bullet_client_->addUserDebugLine({x, y, z_min},
                                                       {x, y, z_max},
                                                       {1,0,0}, 2, 0, -1, owner_id));
    bullet_ids.insert(bullet_client_->addUserDebugLine({x, y, z_min},
                                                       {x_next, y_next, z_min},
                                                       {1,0,0}, 2, 0, -1, owner_id));
    bullet_ids.insert(bullet_client_->addUserDebugLine({x, y, z_max},
                                                       {x_next, y_next, z_max},
                                                       {1,0,0}, 2, 0, -1, owner_id));
  }

  area->setBulletIds(bullet_ids);
}

Entity* AreasPerceptionManager::findAreaOwner(Area* area)
{
  Entity* owner = nullptr;
  for(auto manager : objects_managers_)
  {
    owner = manager->getEntity(area->getOwnerStr());
    if(owner != nullptr)
      return owner;
  }

  for(auto manager : bodyparts_managers_)
  {
    owner = manager->getEntity(area->getOwnerStr());
    if(owner != nullptr)
      return owner;
  }
}

} // namespace owds