#ifndef OWDS_ENTITIESPERCEPTIONMANAGER_H
#define OWDS_ENTITIESPERCEPTIONMANAGER_H

#include <map>
#include <string>
#include <vector>

#include "overworld/BasicTypes/Entity.h"
#include "overworld/Perception/PerceptionModuleBase.h"

#include "overworld/Utility/ShellDisplay.h"


namespace owds {

template<typename T>
class EntitiesPerceptionManager
{
    static_assert(std::is_base_of<Entity,T>::value, "T must be derived from Entity");
public:
    void addPerceptionModule(const std::string& module_name, PerceptionModuleBase_<T>* perception_module);
    PerceptionModuleBase_<T>* getPerceptionModule(const std::string module_name);
    std::vector<std::string> getModulesList();
    std::vector<std::string> getActivatedModulesList();
    std::string getModulesListStr();
    std::string getActivatedModulesListStr();

    bool update();

protected:
    std::map<std::string, T> entities_;
    std::map<std::string, PerceptionModuleBase_<T>* > perception_modules_;

    bool shouldRun();
    virtual void getPercepts(const std::map<std::string, T>& percepts);
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
bool EntitiesPerceptionManager<T>::shouldRun()
{
    for(const auto& module : perception_modules_)
        if(module.second->hasBeenUpdated())
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
            entities_.insert(std::pair<std::string, T>(percept.second.id(), percept.second));
        else
            it->second.updatePose(percept.second.pose(), percept.second.lastStamp());
    }
}

template<typename T>
bool EntitiesPerceptionManager<T>::update()
{
    if(!shouldRun())
        return false;

    for(const auto& module : perception_modules_)
        module.second->accessPercepts([this](const std::map<std::string, T>& percepts){ this->getPercepts(percepts); });

    return true;
}

} // namespace owds

#endif // OWDS_ENTITIESPERCEPTIONMANAGER_H