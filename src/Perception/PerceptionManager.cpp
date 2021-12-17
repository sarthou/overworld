#include "overworld/Perception/PerceptionManager.h"

#include <pluginlib/class_loader.h>

#include "overworld/Utility/ShellDisplay.h"

namespace owds {

PerceptionManager::PerceptionManager(ros::NodeHandle* n, BulletClient* bullet_client)
{
    n_ = n;
    bullet_client_ = bullet_client;
}

PerceptionManager::~PerceptionManager()
{
    robots_manager_.deleteModules();
    objects_manager_.deleteModules();
    humans_manager_.deleteModules();
}

void PerceptionManager::update()
{
    robots_manager_.update();
    objects_manager_.update();
    humans_manager_.update();
}

bool PerceptionManager::applyConfigurationRobot(const std::string& config_path)
{
    ConfigElement modules_list;
    if(applyConfiguration(config_path, modules_list) == false)
        return false;

    auto humans_modules = modules_list["humans"].getElementsKeys();
    auto objects_modules = modules_list["objects"].getElementsKeys();

    // Load humans perception modules

    pluginlib::ClassLoader<PerceptionModuleBase_<BodyPart>> agents_loader("overworld", "owds::PerceptionModuleBase_<owds::BodyPart>");

    for(auto& human_module : humans_modules)
    {
        for(auto& human_module_name : modules_list["humans"][human_module].value())
        {
            auto human_perception_module = agents_loader.createUnmanagedInstance("owds::" + human_module);
            ConfigElement module_config = configuration_[human_module_name];
            human_perception_module->initialize(n_, bullet_client_, robot_bullet_id_, robot_agent_);
            for(auto& param_name : module_config.getElementsKeys())
                human_perception_module->setParameter(param_name, module_config[param_name].value().front());

            if(human_perception_module->closeInitialization() == false)
                return false;

            humans_manager_.addPerceptionModule(human_module_name, human_perception_module);
        }
    }

    // Load objects perception modules

    pluginlib::ClassLoader<PerceptionModuleBase_<Object>> objects_loader("overworld", "owds::PerceptionModuleBase_<owds::Object>");

    for(auto& object_module : objects_modules)
    {
        for(auto& object_module_name : modules_list["objects"][object_module].value())
        {
            auto object_perception_module = objects_loader.createUnmanagedInstance("owds::" + object_module);
            ConfigElement module_config = configuration_[object_module_name];
            object_perception_module->initialize(n_, bullet_client_, robot_bullet_id_, robot_agent_);
            for(auto& param_name : module_config.getElementsKeys())
                object_perception_module->setParameter(param_name, module_config[param_name].value().front());

            if(object_perception_module->closeInitialization() == false)
                return false;

            objects_manager_.addPerceptionModule(object_module_name, object_perception_module);
        }
    }

    return true;
}

bool PerceptionManager::applyConfigurationHuman(const std::string& config_path)
{
    ConfigElement modules_list;
    if(applyConfiguration(config_path, modules_list) == false)
        return false;

    // Load static perception modules

    pluginlib::ClassLoader<PerceptionModuleBase_<Object>> objects_loader("overworld", "owds::PerceptionModuleBase_<owds::Object>");

    std::vector<std::string> objects_modules = {"StaticObjectsPerceptionModule"};

    for(auto& object_module : objects_modules)
    {
        for(auto& object_module_name : modules_list["objects"][object_module].value())
        {
            auto object_perception_module = objects_loader.createUnmanagedInstance("owds::" + object_module);
            ConfigElement module_config = configuration_[object_module_name];
            object_perception_module->initialize(n_, bullet_client_, robot_bullet_id_, robot_agent_);
            for(auto& param_name : module_config.getElementsKeys())
                object_perception_module->setParameter(param_name, module_config[param_name].value().front());

            if(object_perception_module->closeInitialization() == false)
                return false;

            objects_manager_.addPerceptionModule(object_module_name, object_perception_module);
        }
    }

    return true;
}

bool PerceptionManager::applyConfiguration(const std::string& config_path, ConfigElement& modules_list)
{
    if(configuration_.read(config_path) == false)
    {
        ShellDisplay::error("Can not open configuration file: " + config_path);
        return false;
    }

    configuration_.display();

    modules_list = configuration_["modules"];
    if(!modules_list.getElementsKeys().size())
    {
        ShellDisplay::error("No modules defined in the configuration file");
        return false;
    }

    auto robot_modules = modules_list["robot"].getElementsKeys();

    if((robot_modules.size() != 1) || (modules_list["robot"][robot_modules.front()].value().size() != 1))
    {
        ShellDisplay::error("One and only one robot perception module should be provided. " + std::to_string(robot_modules.size()) + " provided");
        return false;
    }

    pluginlib::ClassLoader<PerceptionModuleBase_<BodyPart>> agents_loader("overworld", "owds::PerceptionModuleBase_<owds::BodyPart>");

    auto robot_perception_module = agents_loader.createUnmanagedInstance("owds::" + robot_modules[0]);
    std::string module_name = modules_list["robot"][robot_modules.front()].value().front();
    ConfigElement module_config = configuration_[module_name];
    robot_perception_module->initialize(n_, bullet_client_, -1, nullptr);
    for(auto& param_name : module_config.getElementsKeys())
        robot_perception_module->setParameter(param_name, module_config[param_name].value().front());
    
    if(robot_perception_module->closeInitialization() == false)
        return false;

    robot_name_ = robot_perception_module->getAgentName();
    robot_bullet_id_ = robot_perception_module->getAgentBulletId();

    robots_manager_.addPerceptionModule(module_name, robot_perception_module);

    robot_agent_ = robots_manager_.getAgent(robot_name_);

    return true;
}

} // namesapce owds