#include "overworld/Utility/YamlReader.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"
#include "overworld/BasicTypes/Object.h"

#include <ros/package.h>
#include <iostream>

#include <pluginlib/class_loader.h>

namespace owds {

void load()
{
    pluginlib::ClassLoader<PerceptionModuleBase_<Object>> loader("overworld", "owds::PerceptionModuleBase_<owds::Object>");
    std::vector<std::string> plugins = loader.getDeclaredClasses();
    std::cout << "get " << plugins.size() << " plugins" << std::endl;
    for(auto& pl : plugins)
    {
        std::cout << "- " << pl << std::endl;
        PerceptionModuleBase_<Object>* tmp = loader.createUnmanagedInstance(pl);
    }
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    owds::YamlReader config;

    std::string path = ros::package::getPath("overworld") + "/config/config_example.yaml";

    std::cout << "path = " << path << std::endl;
    if(config.read(path) == false)
        std::cout << "can not open configuration file" << std::endl;
    else
        config.display();

    owds::load();

    return 0;
}