#ifndef OWDS_ENTITIESPERCEPTIONMANAGER_H
#define OWDS_ENTITIESPERCEPTIONMANAGER_H

#include <map>
#include <string>
#include <vector>

#include "overworld/BasicTypes/Entity.h"
#include "overworld/Bullet/BulletClient.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include "overworld/Utility/ShellDisplay.h"

#include <ontologenius/OntologiesManipulator.h>

namespace owds {

template<typename T>
class EntitiesPerceptionManager
{
    static_assert(std::is_base_of<Entity,T>::value, "T must be derived from Entity");
public:
    explicit EntitiesPerceptionManager(ros::NodeHandle* nh): bullet_client_(nullptr), 
                                                             ontos_(OntologiesManipulator(nh)),
                                                             onto_(nullptr)
    {}
    virtual ~EntitiesPerceptionManager();

    void setOwnerAgentName(const std::string& agent_name);
    void setBulletClient(BulletClient* client) { bullet_client_ = client; }

    void addPerceptionModule(const std::string& module_name, PerceptionModuleBase_<T>* perception_module);
    PerceptionModuleBase_<T>* getPerceptionModule(const std::string& module_name) const;
    std::vector<std::string> getModulesList() const;
    std::vector<std::string> getActivatedModulesList() const;
    std::string getModulesListStr() const;
    std::string getActivatedModulesListStr() const;
    void deleteModules();
    const std::map<std::string, T*>& getEntities() const { return entities_; }

    bool update();

protected:
    std::map<std::string, T*> entities_;
    std::unordered_set<std::string> black_listed_entities_;
    std::map<std::string, PerceptionModuleBase_<T>* > perception_modules_;
    BulletClient* bullet_client_;

    std::string myself_agent_name_;
    OntologiesManipulator ontos_;
    OntologyManipulator* onto_;

    bool shouldRun();
    virtual void getPercepts( std::map<std::string, T>& percepts);
    virtual void reasoningOnUpdate() {}

    void updateEntityPose(T* entity, const Pose& pose, const ros::Time& stamp);
    void removeEntityPose(T* entity);

    void addToBullet(T* entity);
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
void EntitiesPerceptionManager<T>::addPerceptionModule(const std::string& module_name, PerceptionModuleBase_<T>* perception_module)
{
    if(perception_modules_.find(module_name) == perception_modules_.end())
        perception_modules_[module_name] = perception_module;
    else
        ShellDisplay::error("A perception module named " + module_name + " is already registered");
}

template<typename T>
PerceptionModuleBase_<T>* EntitiesPerceptionManager<T>::getPerceptionModule(const std::string& module_name) const
{
    if(perception_modules_.find(module_name) != perception_modules_.end())
        return perception_modules_.at(module_name);
    else
        return nullptr;
}

template<typename T>
std::vector<std::string> EntitiesPerceptionManager<T>::getModulesList() const
{
    std::vector<std::string> modules_name;
    std::transform(perception_modules_.cbegin(), perception_modules_.cend(), std::back_inserter(modules_name), [](const auto& it){return it.first;});
    return modules_name;
}

template<typename T>
std::vector<std::string> EntitiesPerceptionManager<T>::getActivatedModulesList() const
{
    std::vector<std::string> modules_name;
    for(const auto& module : perception_modules_)
    {
        if(module.second->isActivated())
            modules_name.push_back(module.first);
    }
    return modules_name;
}

template<typename T>
std::string EntitiesPerceptionManager<T>::getModulesListStr() const
{
    std::string modules_name;
    for(const auto& module : perception_modules_)
    {
        if(modules_name != "")
            modules_name += ", ";
        modules_name += module.first;
    }
    return modules_name;
}

template<typename T>
std::string EntitiesPerceptionManager<T>::getActivatedModulesListStr() const
{
    std::string modules_name;
    for(const auto& module : perception_modules_)
    {
        if(module.second->isActivated())
        {
            if(modules_name != "")
                modules_name += ", ";
            modules_name += module.first;
        }
    }
    return modules_name;
}

template<typename T>
void EntitiesPerceptionManager<T>::deleteModules()
{
    for(const auto& module : perception_modules_)
    {
        module.second->activate(false);
        delete module.second;
    }
    perception_modules_.clear();
}

template<typename T>
bool EntitiesPerceptionManager<T>::shouldRun()
{
    return std::any_of(perception_modules_.begin(), perception_modules_.end(), [](const auto& it){return it.second->isActivated() && it.second->hasBeenUpdated();});
}

template<typename T>
void EntitiesPerceptionManager<T>::getPercepts( std::map<std::string, T>& percepts)
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
    if(!shouldRun())
        return false;

    for(auto& entity : entities_)
        entity.second->setUnseen();

    for(const auto& module : perception_modules_)
        if(module.second->isActivated() && module.second->hasBeenUpdated())
            module.second->accessPercepts([this](std::map<std::string, T>& percepts){ this->getPercepts(percepts); });

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
void EntitiesPerceptionManager<T>::addToBullet(T* entity)
{
    if(black_listed_entities_.find(entity->id()) != black_listed_entities_.end())
        return;

    int visual_id = -1;
    int collision_id = -1;

    switch(entity->getShape().type)
    {
        case SHAPE_CUBE:
        {
            visual_id = bullet_client_->createVisualShapeBox(entity->getShape().scale,
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
	        bullet_client_->changeRgbaColor(obj_id, -1, {1,1,1,1});
	        bullet_client_->changeTexture(obj_id, -1, texture_id);

        }
        bullet_client_->setMass(obj_id, -1, 0); // We force the mass to zero to not have gravity effect
        bullet_client_->setRestitution(obj_id, -1, 0.001);
        bullet_client_->setFrictionAnchor(obj_id, -1, 0.7);
        bullet_client_->setLateralFriction(obj_id, -1, 0.7);
        bullet_client_->setSpinningFriction(obj_id, -1, 0.7);
        bullet_client_->setRollingFriction(obj_id, -1, 0.7);
        bullet_client_->setActivationState(obj_id, eActivationStateDisableSleeping);
        entity->setBulletId(obj_id);
    }
    else
    {
        black_listed_entities_.insert(entity->id());
        ShellDisplay::warning("Entity " + entity->id() + " has been black listed from body creation to prevent future errors");
    }
}

template<typename T>
void EntitiesPerceptionManager<T>::updateToBullet(T* entity)
{
    if(entity->bulletId() != -1)
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
    if(entity->bulletId() != -1)
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