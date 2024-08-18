#ifndef OWDS_ENTITIESPERCEPTIONMANAGER_H
#define OWDS_ENTITIESPERCEPTIONMANAGER_H

#include <map>
#include <ontologenius/OntologiesManipulator.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/Percept.h"
#include "overworld/Bullet/BulletClient.h"
#include "overworld/Perception/DataFusion/DataFusionBase.h"
#include "overworld/Perception/Managers/BasePerceptionManager.h"
#include "overworld/Utils/RosFiles.h"
#include "overworld/Utils/ShellDisplay.h"
#include "overworld/Utils/Wavefront.h"

namespace owds {

  template<typename T>
  class EntitiesPerceptionManager : public BasePerceptionManager<T>
  {
    static_assert(std::is_base_of<Entity, T>::value, "T must be derived from Entity");

  public:
    explicit EntitiesPerceptionManager(ros::NodeHandle* nh) : bullet_client_(nullptr), onto_(nullptr) {}
    virtual ~EntitiesPerceptionManager();

    void setOwnerAgentName(const std::string& agent_name);
    void setBulletClient(BulletClient* client) { bullet_client_ = client; }

    const std::map<std::string, T*>& getEntities() const { return entities_; }
    T* getEntity(const std::string& entity_id) const;

    bool update();

  protected:
    std::map<std::string, T*> entities_;
    std::map<std::string, std::vector<Percept<T>>> aggregated_;
    std::unordered_map<std::string, Percept<T>*> fusioned_percepts_;
    std::unordered_set<std::string> black_listed_entities_;
    BulletClient* bullet_client_;

    std::string myself_agent_name_;
    onto::OntologiesManipulator ontos_;
    onto::OntologyManipulator* onto_;

    virtual void getPercepts(const std::string& module_name, std::map<std::string, Percept<T>>& percepts);
    virtual void reasoningOnUpdate() {}

    void updateEntityPose(T* entity, const Pose& pose, const ros::Time& stamp);
    void removeEntityPose(T* entity);

    bool addToBullet(T* entity);
    void addToBullet(T* entity, int bullet_parent_id);
    void updateToBullet(T* entity);
    void undoInBullet(T* entity);
    T* getEntityFromBulletId(int bullet_id);

    void UpdateAabbs();
  };

  template<typename T>
  EntitiesPerceptionManager<T>::~EntitiesPerceptionManager()
  {
    for(auto& entity : entities_)
      delete entity.second;
    entities_.clear();

    if(onto_ != nullptr)
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
  void EntitiesPerceptionManager<T>::getPercepts(const std::string& module_name, std::map<std::string, Percept<T>>& percepts)
  {
    // This implementation only has test purposes
    for(auto& percept : percepts)
    {
      auto it = entities_.find(percept.second.id());
      if(it == entities_.end())
      {
        T* new_entity = new T(percept.second);
        it = entities_.insert(std::pair<std::string, T*>(percept.second.id(), new_entity)).first;
        addToBullet(it->second);
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
    updateToBullet(entity);
    entity->setSeen();
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::removeEntityPose(T* entity)
  {
    entity->unsetPose();
    updateToBullet(entity);
  }

  template<typename T>
  bool EntitiesPerceptionManager<T>::addToBullet(T* entity)
  {
    if(black_listed_entities_.find(entity->id()) != black_listed_entities_.end())
      return true;

    int visual_id = -1;
    int collision_id = -1;

    switch(entity->getShape().type)
    {
    case SHAPE_CUBE:
    {
      visual_id = bullet_client_->createVisualShapeBox({entity->getShape().scale[0] / 2.,
                                                        entity->getShape().scale[1] / 2.,
                                                        entity->getShape().scale[2] / 2.},
                                                       {entity->getShape().color[0],
                                                        entity->getShape().color[1],
                                                        entity->getShape().color[2],
                                                        1});
      collision_id = bullet_client_->createCollisionShapeBox({entity->getShape().scale[0] / 2.,
                                                              entity->getShape().scale[1] / 2.,
                                                              entity->getShape().scale[2] / 2.});
      break;
    }
    case SHAPE_SPEHERE:
    {
      visual_id = bullet_client_->createVisualShapeSphere(entity->getShape().scale[0],
                                                          {entity->getShape().color[0],
                                                           entity->getShape().color[1],
                                                           entity->getShape().color[2],
                                                           1});
      collision_id = bullet_client_->createCollisionShapeSphere(entity->getShape().scale[0]);
      break;
    }
    case SHAPE_CYLINDER:
    {
      visual_id = bullet_client_->createVisualShapeCylinder(std::min(entity->getShape().scale[0], entity->getShape().scale[1]) / 2.,
                                                            entity->getShape().scale[2],
                                                            {entity->getShape().color[0],
                                                             entity->getShape().color[1],
                                                             entity->getShape().color[2],
                                                             1});
      collision_id = bullet_client_->createCollisionShapeCylinder(std::min(entity->getShape().scale[0], entity->getShape().scale[1]) / 2.,
                                                                  entity->getShape().scale[2]);
      break;
    }
    case SHAPE_MESH:
    {
      visual_id = bullet_client_->createVisualShapeMesh(entity->getShape().visual_mesh_resource,
                                                        entity->getShape().scale,
                                                        {entity->getShape().color[0],
                                                         entity->getShape().color[1],
                                                         entity->getShape().color[2],
                                                         1});
      std::string colision_mesh = entity->getShape().colision_mesh_resource;
      if(colision_mesh == "")
        colision_mesh = entity->getShape().visual_mesh_resource;
      collision_id = bullet_client_->createCollisionShapeMesh(colision_mesh,
                                                              entity->getShape().scale);
      break;
    }
    }

    if((visual_id != -1) && (collision_id != -1))
    {
      auto entity_pose = entity->pose().arrays();
      int obj_id = bullet_client_->createMultiBody(0, collision_id, visual_id, entity_pose.first, entity_pose.second);
      if(entity->getShape().texture != "")
      {
        int texture_id = bullet_client_->loadTexture(entity->getShape().texture);
        bullet_client_->changeRgbaColor(obj_id, -1, {1, 1, 1, 1});
        bullet_client_->changeTexture(obj_id, -1, texture_id);
      }
      bullet_client_->setMass(obj_id, -1, 0); // We force the mass to zero to not have gravity effect
      bullet_client_->setRestitution(obj_id, -1, 0.001);
      bullet_client_->setFrictionAnchor(obj_id, -1, 0.5);
      bullet_client_->setLateralFriction(obj_id, -1, 0.5);
      bullet_client_->setSpinningFriction(obj_id, -1, 0.5);
      bullet_client_->setRollingFriction(obj_id, -1, 0.5);
      bullet_client_->setActivationState(obj_id, eActivationStateDisableSleeping);
      entity->setBulletId(obj_id);
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
  void EntitiesPerceptionManager<T>::addToBullet(T* entity, int bullet_parent_id)
  {
    auto p = bullet_client_->findJointAndLinkIndices(bullet_parent_id);
    // std::unordered_map<std::string, int> joint_name_id = p.first;
    std::unordered_map<std::string, int> links_name_id = p.second;

    auto it = links_name_id.find(entity->id());
    if(it != links_name_id.end())
    {
      entity->setBulletId(bullet_parent_id);
      entity->setBulletLinkId(it->second);
    }
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::updateToBullet(T* entity)
  {
    if((entity->bulletLinkId() == -1) && (entity->bulletId() != -1))
    {
      if(entity->isLocated() == true)
      {
        auto entity_pose = entity->pose().arrays();
        bullet_client_->resetBasePositionAndOrientation(entity->bulletId(),
                                                        entity_pose.first,
                                                        entity_pose.second);
      }
      else
        bullet_client_->resetBasePositionAndOrientation(entity->bulletId(),
                                                        {0.0, 0.0, -100.0},
                                                        {0.0, 0.0, 0.0, 1.0});
    }
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::undoInBullet(T* entity)
  {
    if((entity->bulletLinkId() == -1) && (entity->bulletId() != -1))
    {
      if(entity->isLocated() == true)
      {
        auto entity_pose = entity->pose(1).arrays();
        bullet_client_->resetBasePositionAndOrientation(entity->bulletId(),
                                                        entity_pose.first,
                                                        entity_pose.second);
      }
    }
  }

  template<typename T>
  T* EntitiesPerceptionManager<T>::getEntityFromBulletId(int bullet_id)
  {
    for(auto entity : entities_)
    {
      if(entity.second->bulletLinkId() == -1)
        if(entity.second->bulletId() == bullet_id)
          return entity.second;
    }
    return nullptr;
  }

  template<typename T>
  void EntitiesPerceptionManager<T>::UpdateAabbs()
  {
    for(auto entity : entities_)
      if(entity.second->isLocated() && entity.second->hasShape()) // TODO test if entity has moved
        entity.second->setAabb(bullet_client_->getAABB(entity.second->bulletId()));
  }

} // namespace owds

#endif // OWDS_ENTITIESPERCEPTIONMANAGER_H