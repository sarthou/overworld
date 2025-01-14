#ifndef OWDS_COMMANDS_H
#define OWDS_COMMANDS_H

#include <array>
#include <string>
#include <vector>

namespace owds {

  std::string execCmd(std::string cmd);

  std::string findPackageRos1(const std::string& pkg_name);

  std::string findPackageRos2(const std::string& pkg_name);

  std::string findPackage(const std::string& pkg_name);

  std::vector<std::string> listPackagesRos1();

  std::vector<std::string> listPackagesRos2();

  std::vector<std::string> listPackages();

} // namespace owds

#endif // OWDS_COMMANDS_H