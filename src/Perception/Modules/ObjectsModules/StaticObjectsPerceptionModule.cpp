#include "overworld/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.h"

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <string>
#include <array>
#include <cstddef>

#include "overworld/Utils/YamlReader.h"
#include "overworld/Utils/RosPackage.h"
#include "overworld/Utils/ShellDisplay.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"

namespace owds {

  StaticObjectsPerceptionModule::StaticObjectsPerceptionModule() : ontologies_manipulator_(nullptr),
                                                                   onto_(nullptr)
  {}

  void StaticObjectsPerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
  {
    if(parameter_name == "file")
      config_file_ = parameter_value;
    else
      ShellDisplay::warning("[StaticObjectsPerceptionModule] Unkown parameter " + parameter_name);
  }

  bool StaticObjectsPerceptionModule::closeInitialization()
  {
    ontologies_manipulator_ = new onto::OntologiesManipulator();
    std::string robot_name = robot_agent_->getId();
    if(ontologies_manipulator_->add(robot_name) == false)
    {
      // we first try without waiting for the ontology as it may takes time
      ontologies_manipulator_->waitInit();
      ontologies_manipulator_->add(robot_name);
    }
    onto_ = ontologies_manipulator_->get(robot_name);
    onto_->close();

    if(config_file_.empty() == false)
      return readConfiguration(config_file_);

    return true;
  }

  void StaticObjectsPerceptionModule::addObject(const std::string& name,
                                                const std::string& visual_mesh,
                                                const std::string& colision_mesh,
                                                const std::array<double, 3>& translation,
                                                const std::array<double, 3>& rotation,
                                                const std::array<double, 4>& color,
                                                const std::string& texture)
  {
    Percept<Object> obj(name);
    Shape_t shape;
    shape.type = SHAPE_MESH;
    shape.visual_mesh_resource = visual_mesh;
    shape.colision_mesh_resource = colision_mesh;
    shape.color = color;
    shape.texture = texture;
    obj.setShape(shape);
    // We do not set the mass since these objects are static
    Pose pose(translation, rotation);
    obj.updatePose(pose);
    obj.setStatic();
    updated_ = true;

    ShellDisplay::success("[StaticObjectsPerceptionModule] create shape for " + name);

    percepts_.insert(std::make_pair(obj.id(), obj));
  }

  void StaticObjectsPerceptionModule::addObject(const std::string& name,
                                                const std::array<double, 3>& translation,
                                                const std::array<double, 3>& rotation,
                                                const std::array<double, 3>& scale)
  {
    if(onto_ == nullptr)
    {
      ShellDisplay::error("[StaticObjectsPerceptionModule] no ontology defined to add a new entity");
      return;
    }

    Percept<Object> obj(name);
    Shape_t shape = ontology::getEntityShape(onto_, name);
    if(shape.type == SHAPE_MESH)
    {
      shape.scale = scale;
      obj.setShape(shape);
      Pose pose(translation, rotation);
      obj.updatePose(pose);
      obj.setStatic();
      updated_ = true;

      percepts_.insert(std::make_pair(obj.id(), obj));

      ShellDisplay::success("[StaticObjectsPerceptionModule] create shape for " + name);
    }
    else
      ShellDisplay::warning("[StaticObjectsPerceptionModule] No mesh defined in the ontology for entity \'" + name + "\'");
  }

  void StaticObjectsPerceptionModule::addVisual(const std::string& name,
                                                const std::string& mesh,
                                                const std::array<double, 3>& translation,
                                                const std::array<double, 3>& rotation,
                                                const std::array<double, 3>& scale)
  {
    if(onto_ == nullptr)
    {
      ShellDisplay::error("[StaticObjectsPerceptionModule] no ontology defined to add a new entity");
      return;
    }

    Shape_t shape;

    if(mesh.empty() == false)
    {
      shape.type = SHAPE_MESH;
      shape.visual_mesh_resource = getFullPath(mesh);
    }
    else
      shape = ontology::getEntityShape(onto_, name);

    if(shape.type == SHAPE_MESH)
    {
      Pose pose(translation, rotation);

      urdf::Geometry_t geom;
      geom.scale = ToV3(scale);
      geom.type = urdf::GeometryType_e::geometry_mesh;
      geom.file_name = shape.visual_mesh_resource;

      auto shape_color = shape.color;
      std::array<float, 4> color{(float)shape_color[0], (float)shape_color[1], (float)shape_color[2], (float)shape_color[3]};
      geom.material.diffuse_color_ = color;
      geom.material.specular_color_ = color;
      geom.material.diffuse_texture_ = shape.texture;
      geom.material.normal_texture_ = shape.normal_map;
      geom.material.specular_texture_ = shape.specular_texture;

      /*
      Material material;
      */

      world_client_->createVisualActor({geom}, pose.arrays().first, pose.arrays().second);
    }
    else
      ShellDisplay::warning("[StaticObjectsPerceptionModule] No mesh defined in the ontology for entity \'" + name + "\'");
  }

  bool StaticObjectsPerceptionModule::readConfiguration(std::string path)
  {
    std::string package_pattern = "package://";
    if(path.find(package_pattern) != std::string::npos)
    {
      size_t pose = path.find(package_pattern);
      size_t pose_end_of_name = path.find("/", pose + package_pattern.size());
      std::string full_package = path.substr(pose, pose_end_of_name - pose);
      std::string package_name = path.substr(pose + package_pattern.size(), pose_end_of_name - pose - package_pattern.size());
      std::string package_path = ros::package::getPath(package_name);
      path.replace(path.find(full_package), full_package.size(), package_path);
    }

    YamlReader reader;
    if(reader.read(path))
    {
      bool success=true;
      auto objects_ids = reader.getKeys();

      if(reader.keyExists("include"))
      {
        auto paths = reader["include"].value();
        for(const auto& path : paths)
        {
          if(readConfiguration(path) == false)
            success = false;
        }
      }

      for(auto& id : objects_ids)
      {
        if(id == "include")
          continue;

        double pose_x = 0, pose_y = 0, pose_z = 0;
        double orientation_x = 0, orientation_y = 0, orientation_z = 0;
        double scale_x=1,scale_y=1,scale_z=1;
        auto obj_description = reader[id];

        if(obj_description.keyExists("position"))
        {
          auto position = obj_description["position"];
          if(position.keyExists("x")) pose_x = std::stod(position["x"].value().front());
          if(position.keyExists("y")) pose_y = std::stod(position["y"].value().front());
          if(position.keyExists("z")) pose_z = std::stod(position["z"].value().front());
        }

        if(obj_description.keyExists("orientation"))
        {
          auto orientation = obj_description["orientation"];
          if(orientation.keyExists("x")) orientation_x = std::stod(orientation["x"].value().front());
          if(orientation.keyExists("y")) orientation_y = std::stod(orientation["y"].value().front());
          if(orientation.keyExists("z")) orientation_z = std::stod(orientation["z"].value().front());
        }

        if(obj_description.keyExists("scale"))
        {
          auto scale = obj_description["scale"];
          if(scale.keyExists("x")) scale_x = std::stod(scale["x"].value().front());
          if(scale.keyExists("y")) scale_y = std::stod(scale["y"].value().front());
          if(scale.keyExists("z")) scale_z = std::stod(scale["z"].value().front());
        }

        bool visual_only = false;
        if(obj_description.keyExists("visual_only"))
          visual_only = (obj_description["visual_only"].value().front() == "true");

        std::string mesh;
        if(obj_description.keyExists("mesh"))
          mesh = obj_description["mesh"].value().front();

        if(visual_only == false) // we consider it as a percpet
          addObject(id, {pose_x, pose_y, pose_z}, {orientation_x, orientation_y, orientation_z},{scale_x,scale_y,scale_z});
        else // we directly put it into the world and never speak again about it
          addVisual(id, mesh, {pose_x, pose_y, pose_z}, {orientation_x, orientation_y, orientation_z},{scale_x,scale_y,scale_z});
      }
      return success;
    }
    else
    {
      ShellDisplay::error("[StaticObjectsPerceptionModule] fail to read file \'" + path + "\'");
      return false;
    }
  }

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::StaticObjectsPerceptionModule, owds::PerceptionModuleBase_<owds::Object>)