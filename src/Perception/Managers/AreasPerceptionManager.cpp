#include "overworld/Perception/Managers/AreasPerceptionManager.h"

#define CIRCLE_STEPS 10

namespace owds {

  AreasPerceptionManager::~AreasPerceptionManager()
  {
    for(auto& area : areas_)
      delete area.second;
    areas_.clear();
  }

  Area* AreasPerceptionManager::getArea(const std::string& area_id)
  {
    auto it = areas_.find(area_id);
    if(it != areas_.end())
      return it->second;
    else
      return nullptr;
  }

  bool AreasPerceptionManager::update()
  {
    if((pending_percepts_.size() == 0) && !this->shouldRun())
      return false;

    for(const auto& module : this->perception_modules_)
      if(module.second->isActivated() && module.second->hasBeenUpdated())
        module.second->accessPercepts([this, name = module.first](std::map<std::string, Percept<Area>>& percepts) { this->getPercepts(name, percepts); });

    solvePendingAreas();

    return true;
  }

  void AreasPerceptionManager::drawAreas()
  {
    drawn_ = true;
    for(auto& area : areas_)
      if(area.second->getWorldLineIds().size() == 0)
        addToWorld(area.second);
  }

  void AreasPerceptionManager::undrawAreas()
  {
    drawn_ = false;
    for(auto& area : areas_)
      if(area.second->getWorldLineIds().size() != 0)
        removeFromBullet(area.second);
  }

  void AreasPerceptionManager::getPercepts(const std::string& module_name, std::map<std::string, Percept<Area>>& percepts)
  {
    for(auto& percept : percepts)
    {
      auto it = areas_.find(percept.second.id());
      if(it == areas_.end())
      {
        auto* new_entity = new Area(percept.second);
        if(percept.second.isStatic() == true)
        {
          it = areas_.insert(std::pair<std::string, Area*>(percept.second.id(), new_entity)).first;
          addToWorld(it->second);
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
        addToWorld(it->second);
        solved.insert(it->first);
      }
    }

    for(auto id : solved)
      pending_percepts_.erase(id);
  }

  void AreasPerceptionManager::removeFromBullet(Area* area)
  {
    for(auto id : area->getWorldLineIds())
      world_client_->removeDebugLine(id);
    for(auto id : area->getWorldTextIds())
      world_client_->removeDebugText(id);
    area->setWorldLineIds({});
    area->setWorldTextIds({});
  }

  void AreasPerceptionManager::addToWorld(Area* area)
  {
    if(drawn_ == false)
      return;

    if(area->isCircle())
      addCircleToBullet(area);
    else
      addPolygonToBullet(area);
  }

  void AreasPerceptionManager::addPolygonToBullet(Area* area)
  {
    std::array<float, 3> color = {1, 0, 0};
    int owner_id = -1;
    int owner_link_id = -1;
    if(area->isStatic() == false)
    {
      owner_id = area->getOwner()->bulletId();
      owner_link_id = area->getOwner()->bulletLinkId();
      color = {0, 0, 1};
    }

    std::unordered_set<int> engine_ids;
    auto polygon_points = area->getPolygon().getBasePoints();
    float z_min = area->getZmin();
    float z_max = area->getZmax();
    float mean_x = 0;
    float mean_y = 0;

    std::vector<std::array<float, 3>> positions;
    std::vector<unsigned int> indices; // min pair, max impair
    positions.reserve(polygon_points.size() * 2);
    indices.reserve(polygon_points.size() * 6);

    for(size_t i = 0; i < polygon_points.size(); i++)
    {
      positions.push_back({(float)polygon_points[i].x, (float)polygon_points[i].y, z_min});
      positions.push_back({(float)polygon_points[i].x, (float)polygon_points[i].y, z_max});

      indices.emplace_back(i * 2);
      indices.emplace_back(i * 2 + 1);
      indices.emplace_back(i * 2);
      indices.emplace_back((i + 1) * 2);
      indices.emplace_back(i * 2 + 1);
      indices.emplace_back((i + 1) * 2 + 1);

      mean_x += polygon_points[i].x;
      mean_y += polygon_points[i].y;
    }

    indices[indices.size() - 1] = 1;
    indices[indices.size() - 3] = 0;

    engine_ids.insert(world_client_->addDebugLine(positions, indices, color,
                                                  0, -1, owner_id, owner_link_id));

    mean_x = mean_x / polygon_points.size();
    mean_y = mean_y / polygon_points.size();

    std::unordered_set<int> engine_text_ids;
    engine_text_ids.insert(world_client_->addDebugText(area->id(),
                                                      {mean_x, mean_y, z_max + 0.2f}, 0.4,
                                                      color, 0, -1, owner_id, owner_link_id));

    area->setWorldLineIds(engine_ids);
    area->setWorldTextIds(engine_text_ids);
  }

  void AreasPerceptionManager::addCircleToBullet(Area* area)
  {
    std::array<float, 3> color = {1, 0, 0};
    int owner_id = -1;
    int owner_link_id = -1;
    if(area->isStatic() == false)
    {
      owner_id = area->getOwner()->bulletId();
      owner_link_id = area->getOwner()->bulletLinkId();
      color = {0, 0, 1};
    }

    std::unordered_set<int> engine_ids;

    float angle_step = (2 * M_PI / CIRCLE_STEPS);
    float radius = area->getRadius();
    float z_min = area->getCenterPose().getZ() - area->getHalfHeight();
    float z_max = area->getCenterPose().getZ() + area->getHalfHeight();
    float x_center = area->getCenterPose().getX();
    float y_center = area->getCenterPose().getY();

    std::vector<std::array<float, 3>> positions;
    std::vector<unsigned int> indices; // min pair, max impair
    positions.reserve(CIRCLE_STEPS * 2);
    indices.reserve(CIRCLE_STEPS * 6);

    for(size_t i = 0; i < CIRCLE_STEPS; i++)
    {
      float angle = i * angle_step;
      float x = x_center + radius * std::cos(angle);
      float y = y_center + radius * std::sin(angle);

      positions.push_back({x, y, z_min});
      positions.push_back({x, y, z_max});

      indices.emplace_back(i * 2);
      indices.emplace_back(i * 2 + 1);
      indices.emplace_back(i * 2);
      indices.emplace_back((i + 1) * 2);
      indices.emplace_back(i * 2 + 1);
      indices.emplace_back((i + 1) * 2 + 1);
    }

    indices[indices.size() - 1] = 1;
    indices[indices.size() - 3] = 0;

    engine_ids.insert(world_client_->addDebugLine(positions, indices, color,
                                                  0, -1, owner_id, owner_link_id));

    std::unordered_set<int> engine_text_ids;
    engine_text_ids.insert(world_client_->addDebugText(area->id(),
                                                      {x_center, y_center, z_max + 0.2f}, 0.4,
                                                      color, 0, -1, owner_id, owner_link_id));

    area->setWorldLineIds(engine_ids);
    area->setWorldTextIds(engine_text_ids);;
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
    return nullptr;
  }

} // namespace owds