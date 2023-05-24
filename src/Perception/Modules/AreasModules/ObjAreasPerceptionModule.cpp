#include "overworld/Perception/Modules/AreasModules/ObjAreasPerceptionModule.h"

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include "overworld/Utility/YamlReader.h"
#include "overworld/Utility/Wavefront.h"
#include "overworld/Utility/RosFiles.h"

namespace owds {

ObjAreasPerceptionModule::ObjAreasPerceptionModule() {}

void ObjAreasPerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
{
  if(parameter_name == "file")
    config_file_ = parameter_value;
  else
    ShellDisplay::warning("[ObjAreasPerceptionModule] Unkown parameter " + parameter_name);
}

bool ObjAreasPerceptionModule::closeInitialization()
{
  if(config_file_ != "")
  {
    std::string package_pattern = "package://";
    if(config_file_.find(package_pattern) != std::string::npos)
    {
      size_t pose = config_file_.find(package_pattern);
      size_t pose_end_of_name = config_file_.find("/", pose + package_pattern.size());
      std::string full_package = config_file_.substr(pose, pose_end_of_name - pose);
      std::string package_name = config_file_.substr(pose + package_pattern.size(), pose_end_of_name - pose - package_pattern.size());
      std::string package_path = ros::package::getPath(package_name);
      config_file_.replace(config_file_.find(full_package), full_package.size(), package_path);
    }

    YamlReader reader;
    if(reader.read(config_file_))
    {
      auto areas_ids = reader.getKeys();
      for(auto& id : areas_ids)
      {
        double pose_x = 0, pose_y = 0, pose_z = 0;
        double radius = 0, half_height = 0;
        std::string polygon_path;
        double hysteresis = 0;
        std::string owner;

        auto area_description = reader[id];

        if(area_description.keyExists("pose"))
        {
          auto position = area_description["pose"];
          if(position.keyExists("x")) pose_x = std::stod(position["x"].value().front());
          if(position.keyExists("y")) pose_y = std::stod(position["y"].value().front());
          if(position.keyExists("z")) pose_z = std::stod(position["z"].value().front());
        }

        if(area_description.keyExists("radius")) radius = std::stod(area_description["radius"].value().front());
        if(area_description.keyExists("half_height")) half_height = std::stod(area_description["half_height"].value().front());
        if(area_description.keyExists("polygon_path")) polygon_path = area_description["polygon_path"].value().front();
        if(area_description.keyExists("owner")) owner = area_description["owner"].value().front();
        if(area_description.keyExists("hysteresis")) hysteresis = std::stod(area_description["hysteresis"].value().front());

        if((polygon_path != "") && (radius != 0))
          ShellDisplay::warning("[ObjAreasPerceptionModule] " + id + " is define by a polygon but a radius is defined. The later will be ignored");
        if(half_height == 0)
        {
          ShellDisplay::error("[ObjAreasPerceptionModule] " + id + " has no height (use half_height parameter).");
          continue;
        }

        if(polygon_path == "")
          addCircle(id, {pose_x, pose_y, pose_z}, radius, half_height, hysteresis, owner);
        else
          addPolygon(id, polygon_path, {pose_x, pose_y, pose_z}, pose_z - half_height, pose_z + half_height, hysteresis, owner);
      }
    }
    else
    {
      ShellDisplay::error("[ObjAreasPerceptionModule] fail to read file \'" + config_file_ + "\'");
      return false;
    }
  }

  return true;
}

void ObjAreasPerceptionModule::addCircle(const std::string& id, const std::array<double, 3>& pose, double radius, double half_height, double hysteresis, const std::string& owner)
{
  Area area(id, Pose({pose[0], pose[1], pose[2]}, {0,0,0,0}), radius, half_height);
  area.setHysteresis(hysteresis);
  area.setOwnerStr(owner);

  updated_ = true;

  percepts_.insert(std::make_pair(area.id(), area));
}

void ObjAreasPerceptionModule::addPolygon(const std::string& id, const std::string& polygon_path, const std::array<double, 3>& pose, double z_min, double z_max, double hysteresis, const std::string& owner)
{
  std::string full_path = getFullPath(polygon_path);
  auto vertexes = wavefront::getVertexes(full_path);
  std::vector<point_t> points;
  for(auto vertex : vertexes)
    points.emplace_back(vertex[0] + pose[0], vertex[1] + pose[1]);
  Polygon polygon(points);
  Area area(id, polygon, z_min, z_max - z_min);
  area.setHysteresis(hysteresis);
  area.setOwnerStr(owner);

  updated_ = true;

  percepts_.insert(std::make_pair(area.id(), area));
}

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::ObjAreasPerceptionModule, owds::PerceptionModuleBase_<owds::Area>)