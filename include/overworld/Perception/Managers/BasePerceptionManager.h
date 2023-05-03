#ifndef OWDS_BASEPERCEPTIONMANAGER_H
#define OWDS_BASEPERCEPTIONMANAGER_H

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include "overworld/Utility/ShellDisplay.h"

namespace owds {

template<typename T>
class BasePerceptionManager
{
public:
  BasePerceptionManager() = default;

  void addPerceptionModule(const std::string& module_name, PerceptionModuleBase_<T>* perception_module);
  PerceptionModuleBase_<T>* getPerceptionModule(const std::string& module_name) const;
  std::vector<std::string> getModulesList() const;
  std::vector<std::string> getActivatedModulesList() const;
  std::string getModulesListStr() const;
  std::string getActivatedModulesListStr() const;
  void deleteModules();

  std::map<std::string, PerceptionModuleBase_<T>* > perception_modules_;
};

template<typename T>
void BasePerceptionManager<T>::addPerceptionModule(const std::string& module_name, PerceptionModuleBase_<T>* perception_module)
{
    if(perception_modules_.find(module_name) == perception_modules_.end())
        perception_modules_[module_name] = perception_module;
    else
        ShellDisplay::error("A perception module named " + module_name + " is already registered");
}

template<typename T>
PerceptionModuleBase_<T>* BasePerceptionManager<T>::getPerceptionModule(const std::string& module_name) const
{
    if(perception_modules_.find(module_name) != perception_modules_.end())
        return perception_modules_.at(module_name);
    else
        return nullptr;
}

template<typename T>
std::vector<std::string> BasePerceptionManager<T>::getModulesList() const
{
    std::vector<std::string> modules_name;
    std::transform(perception_modules_.cbegin(), perception_modules_.cend(), std::back_inserter(modules_name), [](const auto& it){return it.first;});
    return modules_name;
}

template<typename T>
std::vector<std::string> BasePerceptionManager<T>::getActivatedModulesList() const
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
std::string BasePerceptionManager<T>::getModulesListStr() const
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
std::string BasePerceptionManager<T>::getActivatedModulesListStr() const
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
void BasePerceptionManager<T>::deleteModules()
{
    for(const auto& module : perception_modules_)
    {
        module.second->activate(false);
        delete module.second;
    }
    perception_modules_.clear();
}

} // namespace owds

#endif // OWDS_BASEPERCEPTIONMANAGER_H