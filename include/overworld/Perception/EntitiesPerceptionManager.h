#ifndef OWDS_ENTITIESPERCEPTIONMANAGER_H
#define OWDS_ENTITIESPERCEPTIONMANAGER_H

#include <map>
#include <string>
#include <vector>

#include "overworld/BasicTypes/Entity.h"
#include "overworld/Bullet/BulletClient.h"
#include "overworld/Perception/PerceptionModuleBase.h"

#include "overworld/Utility/ShellDisplay.h"


namespace owds {

template<typename T>
class EntitiesPerceptionManager
{
    static_assert(std::is_base_of<Entity,T>::value, "T must be derived from Entity");
public:
    void setBulletClient(BulletClient* client) { bullet_client_ = client; }

    void addPerceptionModule(const std::string& module_name, PerceptionModuleBase_<T>* perception_module);
    PerceptionModuleBase_<T>* getPerceptionModule(const std::string module_name);
    std::vector<std::string> getModulesList();
    std::vector<std::string> getActivatedModulesList();
    std::string getModulesListStr();
    std::string getActivatedModulesListStr();
    void deleteModules();
    inline std::map<std::string, T> getEntities() { return entities_; }

    bool update();

protected:
    std::map<std::string, T> entities_;
    std::map<std::string, PerceptionModuleBase_<T>* > perception_modules_;
    BulletClient* bullet_client_;

    bool shouldRun();
    virtual void getPercepts(const std::map<std::string, T>& percepts);
    virtual void reasoningOnUpdate() {}

    void updateEntityPose(T& entity, const Pose& pose, const ros::Time& stamp);
    void removeEntityPose(T& entity);

    void addToBullet(T& entity);
    void updateToBullet(T& entity);
};

template<typename T>
void EntitiesPerceptionManager<T>::addPerceptionModule(const std::string& module_name, PerceptionModuleBase_<T>* perception_module)
{
    if(perception_modules_.find(module_name) == perception_modules_.end())
        perception_modules_[module_name] = perception_module;
    else
        ShellDisplay::error("A perception module named " + module_name + " is already registered");
}

template<typename T>
PerceptionModuleBase_<T>* EntitiesPerceptionManager<T>::getPerceptionModule(const std::string module_name)
{
    if(perception_modules_.find(module_name) == perception_modules_.end())
        return perception_modules_[module_name];
    else
        return nullptr;
}

template<typename T>
std::vector<std::string> EntitiesPerceptionManager<T>::getModulesList()
{
    std::vector<std::string> modules_name;
    for(const auto& module : perception_modules_)
        modules_name.push_back(module.first);
    return modules_name;
}

template<typename T>
std::vector<std::string> EntitiesPerceptionManager<T>::getActivatedModulesList()
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
std::string EntitiesPerceptionManager<T>::getModulesListStr()
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
std::string EntitiesPerceptionManager<T>::getActivatedModulesListStr()
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
    for(const auto& module : perception_modules_)
        if(module.second->isActivated() && module.second->hasBeenUpdated())
            return true;
    return false;
}

template<typename T>
void EntitiesPerceptionManager<T>::getPercepts(const std::map<std::string, T>& percepts)
{
    // This implementation only has test purposes
    for(auto& percept : percepts)
    {
        auto it = entities_.find(percept.second.id());
        if(it == entities_.end())
        {
            it = entities_.insert(std::pair<std::string, T>(percept.second.id(), percept.second)).first;
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

    for(const auto& module : perception_modules_)
        if(module.second->isActivated() && module.second->hasBeenUpdated())
            module.second->accessPercepts([this](const std::map<std::string, T>& percepts){ this->getPercepts(percepts); });

    reasoningOnUpdate();

    return true;
}

template<typename T>
void EntitiesPerceptionManager<T>::updateEntityPose(T& entity, const Pose& pose, const ros::Time& stamp)
{
    entity.updatePose(pose, stamp);
    updateToBullet(entity);
}

template<typename T>
void EntitiesPerceptionManager<T>::removeEntityPose(T& entity)
{
    entity.unsetPose();
    updateToBullet(entity);
}

template<typename T>
void EntitiesPerceptionManager<T>::addToBullet(T& entity)
{
    int visual_id = -1;
    int collision_id = -1;

    switch(entity.getShape().type)
    {
        case SHAPE_CUBE:
        {
            visual_id = bullet_client_->createVisualShapeBox(entity.getShape().scale,
                                                             {entity.getShape().color[0],
                                                              entity.getShape().color[1],
                                                              entity.getShape().color[3],
                                                              1});
            collision_id = bullet_client_->createCollisionShapeBox(entity.getShape().scale);
            break;
        }
        case SHAPE_SPEHERE:
        {
            visual_id = bullet_client_->createVisualShapeSphere(entity.getShape().scale[0],
                                                                {entity.getShape().color[0],
                                                                 entity.getShape().color[1],
                                                                 entity.getShape().color[3],
                                                                 1});
            collision_id = bullet_client_->createCollisionShapeSphere(entity.getShape().scale[0]);
            break;
        }
        case SHAPE_CYLINDER:
        {
            visual_id = bullet_client_->createVisualShapeCylinder(entity.getShape().scale[0],
                                                                  entity.getShape().scale[1],
                                                                  {entity.getShape().color[0],
                                                                   entity.getShape().color[1],
                                                                   entity.getShape().color[3],
                                                                   1});
            collision_id = bullet_client_->createCollisionShapeCylinder(entity.getShape().scale[0],
                                                                        entity.getShape().scale[1]);
            break;
        }
        case SHAPE_MESH:
        {
            visual_id = bullet_client_->createVisualShapeMesh(entity.getShape().mesh_resource,
                                                              entity.getShape().scale,
                                                              {entity.getShape().color[0],
                                                               entity.getShape().color[1],
                                                               entity.getShape().color[3],
                                                               1});
            collision_id = bullet_client_->createCollisionShapeMesh(entity.getShape().mesh_resource,
                                                                    entity.getShape().scale);
            break;
        }
    }

    if((visual_id != -1) && (collision_id != -1))
    {
        auto entity_pose = entity.pose().arrays();
        int obj_id = bullet_client_->createMultiBody(0, collision_id, visual_id, entity_pose.first, entity_pose.second);
        entity.setBulletId(obj_id);
    }
}

template<typename T>
void EntitiesPerceptionManager<T>::updateToBullet(T& entity)
{
    if(entity.bulletId() != -1)
    {
        auto entity_pose = entity.pose().arrays();
        if(entity.isLocated() == false)
            entity_pose.first[2] -= 100;
        bullet_client_->resetBasePositionAndOrientation(entity.bulletId(),
                                                        entity_pose.first,
                                                        entity_pose.second);
    }
}

} // namespace owds

#endif // OWDS_ENTITIESPERCEPTIONMANAGER_H