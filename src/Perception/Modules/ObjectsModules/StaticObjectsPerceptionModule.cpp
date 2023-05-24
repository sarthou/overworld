#include "overworld/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.h"

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include "overworld/Utility/YamlReader.h"

#include <string>

namespace owds {

StaticObjectsPerceptionModule::StaticObjectsPerceptionModule() : ontologies_manipulator_(nullptr),
                                                                 onto_(nullptr)
{

}

void StaticObjectsPerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
{
    if(parameter_name == "file")
        config_file_ = parameter_value;
    else
        ShellDisplay::warning("[StaticObjectsPerceptionModule] Unkown parameter " + parameter_name);
}

bool StaticObjectsPerceptionModule::closeInitialization()
{
    ontologies_manipulator_ = new OntologiesManipulator(n_);
    std::string robot_name = robot_agent_->getId();
    if(ontologies_manipulator_->add(robot_name) == false)
    {
        // we first try without waiting for the ontology as it may takes time
        ontologies_manipulator_->waitInit();
        ontologies_manipulator_->add(robot_name);
    }
    onto_ = ontologies_manipulator_->get(robot_name);
    onto_->close();

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
            auto objects_ids = reader.getKeys();
            for(auto& id : objects_ids)
            {
                double pose_x = 0, pose_y = 0, pose_z = 0;
                double orientation_x = 0, orientation_y = 0, orientation_z = 0;
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

                addObject(id, {pose_x, pose_y, pose_z}, {orientation_x, orientation_y, orientation_z});
            }
        }
        else
        {
            ShellDisplay::error("[StaticObjectsPerceptionModule] fail to read file \'" + config_file_ + "\'");
            return false;
        }
    }

    return true;
}

void StaticObjectsPerceptionModule::addObject(const std::string& name,
                                              const std::string& visual_mesh,
                                              const std::string& colision_mesh,
                                              const std::array<double,3>& translation,
                                              const std::array<double,3>& rotation,
                                              const std::array<double,3>& color,
                                              const std::string& texture)
{
    Object obj(name);
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
                                              const std::array<double,3>& translation,
                                              const std::array<double,3>& rotation)
{
    if(onto_ == nullptr)
    {
        ShellDisplay::error("[StaticObjectsPerceptionModule] no ontology defined to add a new entity");
        return;
    }

    Object obj(name);
    Shape_t shape = ontology::getEntityShape(onto_, name);
    if(shape.type == SHAPE_MESH)
    {
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

} //namespace owds

PLUGINLIB_EXPORT_CLASS(owds::StaticObjectsPerceptionModule, owds::PerceptionModuleBase_<owds::Object>)