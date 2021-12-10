#include "overworld/Perception/PerceptionConfiguration.h"

#include <ros/package.h>
#include <iostream>

int main()
{
    owds::PerceptionConfiguration config;

    std::string path = ros::package::getPath("overworld") + "/config/config_example.yaml";

    std::cout << "path = " << path << std::endl;
    if(config.read(path) == false)
        std::cout << "can not open configuration file" << std::endl;
    else
        config.display();

    return 0;
}