#include "overworld/Utility/RosFiles.h"

#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <streambuf>

namespace owds {

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

      size_t pattern_pose;
      while((pattern_pose = urdf_str.find(full_package)) != std::string::npos)
      {
        urdf_str.replace(pattern_pose, full_package.size(), package_path);
      }
    }

    std::string mass_pattern_begin = "<mass ";
    std::string mass_pattern_end = "/>";
    size_t search_pose = 0;
    while(urdf_str.find(mass_pattern_begin, search_pose) != std::string::npos)
    {
      size_t pose = urdf_str.find(mass_pattern_begin, search_pose);
      size_t pose_end = urdf_str.find(mass_pattern_end, pose + mass_pattern_begin.size());
      search_pose = pose_end + mass_pattern_end.size();

      urdf_str.replace(pose, pose_end + mass_pattern_end.size() - pose, "<mass value=\"0.0\"/>");
    }

    size_t last = urdf_path.find_last_of("/");
    std::string urdf_name = urdf_path.substr(last + 1);
    std::ofstream out_file;
    out_file.open(urdf_folder + "/" + urdf_name + ".owds");
    out_file << urdf_str;
    out_file.close();

    std::cout << "create " << urdf_name << ".owds file" << std::endl;

    return urdf_folder + "/" + urdf_name + ".owds";
  }

  std::string getFullPath(const std::string& file_name)
  {
    std::string package_pattern = "package://";

    if(file_name.find(package_pattern) != std::string::npos)
    {
      size_t pose = file_name.find(package_pattern);
      size_t pose_end_of_name = file_name.find("/", pose + package_pattern.size());
      std::string full_package = file_name.substr(pose, pose_end_of_name - pose);
      std::string package_name = file_name.substr(pose + package_pattern.size(), pose_end_of_name - pose - package_pattern.size());
      std::string package_path = ros::package::getPath(package_name);

      std::string full_file_name = file_name;
      full_file_name.replace(pose, full_package.size(), package_path);
      return full_file_name;
    }
    else
      return file_name;
  }

} // namespace owds