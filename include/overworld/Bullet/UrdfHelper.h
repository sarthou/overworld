#ifndef OWDS_URDFHELPER_H
#define OWDS_URDFHELPER_H

#include <string>
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>

#include <ros/package.h>

std::string createLocalUrdf(std::string urdf_path, const std::string& urdf_folder)
{
    if(urdf_path.find("/") == std::string::npos)
        urdf_path = urdf_folder + "/" + urdf_path;
    std::ifstream t(urdf_path);
    std::string urdf_str((std::istreambuf_iterator<char>(t)),
                          std::istreambuf_iterator<char>());

    std::string package_pattern = "package://";

    while(urdf_str.find(package_pattern) != std::string::npos)
    {
        size_t pose = urdf_str.find(package_pattern);
        size_t pose_end_of_name = urdf_str.find("/", pose + package_pattern.size());
        std::string full_package = urdf_str.substr(pose, pose_end_of_name - pose);
        std::string package_name = urdf_str.substr(pose + package_pattern.size(), pose_end_of_name - pose - package_pattern.size());
        std::string package_path = ros::package::getPath(package_name);

        std::cout << "full_package = " << full_package << std::endl;
        std::cout << "package_name = " << package_name << std::endl;
        std::cout << "package_path = " << package_path << std::endl;

        size_t pattern_pose = std::string::npos;
        while((pattern_pose = urdf_str.find(full_package)) != std::string::npos)
        {
            urdf_str.replace(pattern_pose, full_package.size(), package_path);
        }
    }


    size_t last = urdf_path.find_last_of("/");
    std::string urdf_name = urdf_path.substr(last + 1);
    std::ofstream out_file;
    out_file.open(urdf_folder + "/" + urdf_name + ".owds");
    out_file << urdf_str;
    out_file.close();

    return urdf_folder + "/" + urdf_name + ".owds";
}

#endif // OWDS_URDFHELPER_H