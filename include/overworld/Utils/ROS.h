#ifndef OWDS_HELPER_ROS_H
#define OWDS_HELPER_ROS_H

#include <cassert>
#include <filesystem>
#include <string>

#include "overworld/Compat/ROS.h"

namespace owds {

  inline std::string rosPkgPathToPath(const std::string& filename)
  {
    const auto url_handler = filename.substr(0, 10);

    assert(url_handler == "package://");

    const auto total_path = filename.substr(10);

    auto parent_path = std::filesystem::path(total_path);
    while(parent_path.has_parent_path())
    {
      parent_path = parent_path.parent_path();
    }

    const auto assets_path = compat::owds_ros::getShareDirectory(parent_path.string());
    const auto relative_path = total_path.substr(parent_path.string().size() + 1);

    return assets_path + "/" + relative_path;
  }

} // namespace owds

#endif // OWDS_HELPER_ROS_H
