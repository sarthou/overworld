#ifndef OWDS_ROSFILES_H
#define OWDS_ROSFILES_H

#include <string>

namespace owds {

  std::string createLocalUrdf(std::string urdf_path, const std::string& urdf_folder);

  std::string getFullPath(const std::string& file_name);

} // namespace owds

#endif // OWDS_ROSFILES_H