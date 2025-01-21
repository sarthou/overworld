#ifndef OWDS_HELPER_ROS_H
#define OWDS_HELPER_ROS_H

#include <cassert>
#include <filesystem>
#include <string>

#include "overworld/Utils/RosPackage.h"

namespace owds {

  inline std::string rosPkgPathToPath(const std::string& filename)
  {
    const auto url_handler = filename.substr(0, 10);

    if(url_handler != "package://")
      return filename;

    const auto total_path = filename.substr(10);

    auto parent_path = std::filesystem::path(total_path);
    while(parent_path.has_parent_path())
    {
      parent_path = parent_path.parent_path();
    }

    const auto assets_path = findPackage(parent_path.string());
    const auto relative_path = total_path.substr(parent_path.string().size() + 1);

    return assets_path + "/" + relative_path;
  }

} // namespace owds

#endif // OWDS_HELPER_ROS_H
