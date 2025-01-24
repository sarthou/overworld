#ifndef OWDS_ENTITIESPERCEPTIONMANAGER_H
#define OWDS_ENTITIESPERCEPTIONMANAGER_H

#include <map>
#include <ontologenius/OntologiesManipulator.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <array>

#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/Percept.h"
#include "overworld/BasicTypes/Sensors/SensorBase.h"
#include "overworld/Engine/Engine.h"
#include "overworld/Perception/DataFusion/DataFusionBase.h"
#include "overworld/Perception/Managers/BasePerceptionManager.h"
#include "overworld/Utils/RosPackage.h"
#include "overworld/Utils/ShellDisplay.h"
#include "overworld/Utils/Wavefront.h"

namespace owds {

  template<typename T>
  class EntitiesPerceptionManager : public BasePerceptionManager<T>
  {
    static_assert(std::is_base_of<Entity, T>::value, "T must be derived from Entity");

  public:
    explicit EntitiesPerceptionManager() : world_client_(nullptr), onto_(nullptr) {}
    virtual ~EntitiesPerceptionManager();

    void setOwnerAgentName(const std::string& agent_name);
    void setWorldClient(WorldEngine* world_client) { world_client_ = world_client; }

    const std::map<std::string, T*>& getEntities() const { return entities_; }
    T* getEntity(const std::string& entity_id) const;

    bool update();

  protected:
    std::map<std::string, T*> entities_;
    std::map<std::string, std::map<std::string, Percept<T>>> aggregated_;
    std::map<std::string, std::map<std::string, std::set<std::string>>> entities_percetion_register_;
    std::unordered_map<std::string, Percept<T>*> fusioned_percepts_;
    std::unordered_set<std::string> black_listed_entities_;
    WorldEngine* world_client_;

    std::string myself_agent_name_;
    onto::OntologiesManipulator ontos_;
    onto::OntologyManipulator* onto_;

    void fusionAggregated(const std::string& entity_id, const std::string& module_name, const Percept<T>& percept);
    void fusionRegister(const std::string& entity_id, const std::string& sensor_id, const std::string& module_name);
    virtual void getPercepts(const std::string& module_name, std::map<std::string, Percept<T>>& percepts);
    virtual void reasoningOnUpdate() {}

    void updateEntityPose(T* entity, const Pose& pose, const ros::Time& stamp);
    void removeEntityPose(T* entity);

    bool addToWorld(Sensor* sensor);
    bool addToWorld(T* entity);
    void addToWorld(T* entity, int bullet_parent_id);
    void updateToEngine(T* entity);
    void undoInEngine(T* entity);
    T* getEntityFromWorldId(int engine_id);

    void updateAabbs();
  };

  template<typename T>
  EntitiesPerceptionManager<T>::~EntitiesPerceptionManager()
  {
    for(auto& entity : entities_)
      delete entity.second;
    entities_.clear();

    delete onto_;
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::setOwnerAgentName(const std::string& agent_name)
  {
    myself_agent_name_ = agent_name;
    ontos_.waitInit();
    ontos_.add(myself_agent_name_);
    onto_ = ontos_.get(myself_agent_name_);
    onto_->close();
  }

  template<typename T>
  T* EntitiesPerceptionManager<T>::getEntity(const std::string& entity_id) const
  {
    auto it = entities_.find(entity_id);
    if(it != entities_.end())
      return it->second;
    else
      return nullptr;
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::fusionRegister(const std::string& entity_id, const std::string& sensor_id, const std::string& module_name)
  {
    if(sensor_id.empty() == false) // we avoid problems with static object
    {
      auto it = entities_percetion_register_.find(entity_id);
      if(it == entities_percetion_register_.end())
        entities_percetion_register_.emplace(entity_id, std::map<std::string, std::set<std::string>>{{sensor_id, {module_name}}});
      else
      {
        auto inner_it = it->second.find(sensor_id);
        if(inner_it == it->second.end())
          it->second.emplace(sensor_id, std::set<std::string>{module_name});
        else
          inner_it->second.insert(module_name);
      }
    }
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::fusionAggregated(const std::string& entity_id, const std::string& module_name, const Percept<T>& percept)
  {
    auto it = aggregated_.find(entity_id);
    if(it == aggregated_.end())
    {
      aggregated_.emplace(entity_id, std::map<std::string, Percept<T>>{{module_name, percept}});
    }
    else
    {
      auto inner_it = it->second.find(module_name);
      if(inner_it == it->second.end())
        it->second.emplace(module_name, percept);
      else
        inner_it->second = percept;
    }
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::getPercepts(const std::string& module_name, std::map<std::string, Percept<T>>& percepts)
  {
    (void)module_name;
    // This implementation only has test purposes
    for(auto& percept : percepts)
    {
      auto it = entities_.find(percept.second.id());
      if(it == entities_.end())
      {
        T* new_entity = new T(percept.second);
        it = entities_.insert(std::pair<std::string, T*>(percept.second.id(), new_entity)).first;
        addToWorld(it->second);
      }

      updateEntityPose(it->second, percept.second.pose(), percept.second.lastStamp());
    }
  }

  template<typename T>
  bool EntitiesPerceptionManager<T>::update()
  {
    if(!this->shouldRun())
      return false;

    for(auto& entity : entities_)
      entity.second->setUnseen();

    for(const auto& module : this->perception_modules_)
      if(module.second->isActivated() /*&& module.second->hasBeenUpdated()*/) // TODO try a way to not check all modules
        module.second->accessPercepts([this, name = module.first](std::map<std::string, Percept<T>>& percepts) { this->getPercepts(name, percepts); });

    reasoningOnUpdate();

    return true;
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::updateEntityPose(T* entity, const Pose& pose, const ros::Time& stamp)
  {
    entity->updatePose(pose, stamp);
    updateToEngine(entity);
    entity->setSeen();
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::removeEntityPose(T* entity)
  {
    auto percept = fusioned_percepts_.find(entity->id());
    if(percept != fusioned_percepts_.end())
      percept->second->unsetPose();
    entity->unsetPose();
    updateToEngine(entity);
  }

  template<typename T>
  bool EntitiesPerceptionManager<T>::addToWorld(Sensor* sensor)
  {
    if(sensor->getWorldSegmentationId() != -1)
      return true;

    auto fov = sensor->getFieldOfView();
    int id = world_client_->addCamera(300 * fov.getRatioOpenGl(), 300, fov.getRatioOpenGl(), CameraView_e::segmented_view, fov.getClipNear(), fov.getClipFar());
    sensor->setWorldSegmentationId(id);
    return id != -1;
  }

  template<typename T>
  bool EntitiesPerceptionManager<T>::addToWorld(T* entity)
  {
    if(black_listed_entities_.find(entity->id()) != black_listed_entities_.end())
      return true;

    urdf::Geometry_t collision_geom;
    urdf::Geometry_t visual_geom;

    switch(entity->getShape().type)
    {
    case SHAPE_CUBE:
    {
      collision_geom.type = urdf::geometry_box;
      collision_geom.scale = {entity->getShape().scale[0] / 2.,
                              entity->getShape().scale[1] / 2.,
                              entity->getShape().scale[2] / 2.};

      visual_geom = collision_geom;

      break;
    }
    case SHAPE_SPEHERE:
    {
      collision_geom.type = urdf::geometry_sphere;
      collision_geom.scale = {entity->getShape().scale[0],
                              0.f, 0.f};

      visual_geom = collision_geom;

      break;
    }
    case SHAPE_CYLINDER:
    {
      collision_geom.type = urdf::geometry_cylinder;
      collision_geom.scale = {std::min(entity->getShape().scale[0], entity->getShape().scale[1]) / 2.,
                              entity->getShape().scale[2],
                              0.f};

      visual_geom = collision_geom;

      break;
    }
    case SHAPE_MESH:
    {
      visual_geom.type = urdf::geometry_mesh;

      visual_geom.file_name = entity->getShape().visual_mesh_resource;
      visual_geom.scale = {entity->getShape().scale[0],
                           entity->getShape().scale[1],
                           entity->getShape().scale[2]};

      collision_geom = visual_geom;
      std::string colision_mesh = entity->getShape().colision_mesh_resource;
      if(colision_mesh.empty() == false)
        collision_geom.file_name = colision_mesh;

      break;
    }
    default:
      return false;
    }

    if(visual_geom.type != urdf::geometry_none)
    {
      auto shape_color = entity->getShape().color;
      std::array<float, 4> color{(float)shape_color[0], (float)shape_color[1], (float)shape_color[2], (float)shape_color[3]};
      visual_geom.material.diffuse_color_ = color;
      visual_geom.material.specular_color_ = color;
      visual_geom.material.diffuse_texture_ = entity->getShape().texture;

      auto entity_pose = entity->pose().arrays();
      int obj_id = -1;
      if(entity->isStatic())
        obj_id = world_client_->createStaticActor(collision_geom, {visual_geom},
                                                  entity_pose.first, entity_pose.second);
      else
        obj_id = world_client_->createActor(collision_geom, {visual_geom},
                                            entity_pose.first, entity_pose.second);

      world_client_->setMass(obj_id, -1, 0); // We force the mass to zero to not have gravity effect
      world_client_->setRestitution(obj_id, -1, 0.001);
      world_client_->setStaticFriction(obj_id, -1, 0.5);
      world_client_->setDynamicFriction(obj_id, -1, 0.5);

      entity->setWorldId(obj_id);

      return true;
    }
    else
    {
      black_listed_entities_.insert(entity->id());
      ShellDisplay::warning("Entity " + entity->id() + " has been black listed from body creation to prevent future errors");
      return false;
    }
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::addToWorld(T* entity, int bullet_parent_id)
  {
    int link_id = world_client_->getLinkId(bullet_parent_id, entity->id());
    if(link_id != -1)
    {
      entity->setWorldId(bullet_parent_id);
      entity->setBulletLinkId(link_id);
    }
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::updateToEngine(T* entity)
  {
    if((entity->bulletLinkId() == -1) && (entity->worldId() != -1))
    {
      if(entity->isLocated() == true)
      {
        auto entity_pose = entity->pose().arrays();
        world_client_->setBasePositionAndOrientation(entity->worldId(),
                                                     entity_pose.first,
                                                     entity_pose.second);
      }
      else
        world_client_->setBasePositionAndOrientation(entity->worldId(),
                                                     {0.0, 0.0, -100.0},
                                                     {0.0, 0.0, 0.0, 1.0});
    }
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::undoInEngine(T* entity)
  {
    if((entity->bulletLinkId() == -1) && (entity->worldId() != -1))
    {
      if(entity->isLocated() == true)
      {
        auto entity_pose = entity->pose(1).arrays();
        world_client_->setBasePositionAndOrientation(entity->worldId(),
                                                     entity_pose.first,
                                                     entity_pose.second);
      }
    }
  }

  template<typename T>
  T* EntitiesPerceptionManager<T>::getEntityFromWorldId(int engine_id)
  {
    for(auto entity : entities_)
    {
      if(entity.second->bulletLinkId() == -1)
        if(entity.second->worldId() == engine_id)
          return entity.second;
    }
    return nullptr;
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::updateAabbs()
  {
    for(auto entity : entities_)
      if(entity.second->isLocated() && entity.second->hasShape()) // TODO test if entity has moved
        entity.second->setAabb(world_client_->getAABB(entity.second->worldId()));
  }

} // namespace owds

#endif // OWDS_ENTITIESPERCEPTIONMANAGER_H