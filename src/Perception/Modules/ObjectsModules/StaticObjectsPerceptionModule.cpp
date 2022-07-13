#include "overworld/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.h"

#include <pluginlib/class_list_macros.h>

namespace owds {

StaticObjectsPerceptionModule::StaticObjectsPerceptionModule() : ontologies_manipulator_(nullptr),
                                                                 onto_(nullptr)
{

}

bool StaticObjectsPerceptionModule::closeInitialization()
{
    ontologies_manipulator_ = new OntologiesManipulator(n_);
    ontologies_manipulator_->waitInit();
    std::string robot_name = robot_agent_->getId();
    ontologies_manipulator_->add(robot_name);
    onto_ = ontologies_manipulator_->get(robot_name);
    onto_->close();

    addObject("adream_walls", {0,0,0}, {0,0,1.57});
    addObject("adream_floor", {0,0,0}, {0,0,1.57});
    addObject("adream_window_h10_1", {0,0,0}, {0,0,1.57});
    addObject("adream_window_h10_2", {0,0,0}, {0,0,1.57});
    addObject("adream_window_h20_1", {0,0,0}, {0,0,1.57});
    addObject("adream_window_h20_2", {0,0,0}, {0,0,1.57});
    addObject("adream_window_h20_3", {0,0,0}, {0,0,1.57});
    addObject("adream_appartment", {0,0,0}, {0,0,1.57});
    addObject("adream_door_h21_1", {1.03963,-0.00339,1.0725}, {0,0,1.57});
    addObject("adream_door_h21_2", {5.09035,18.4801,1.0725}, {0,0,1.57});
    addObject("adream_door_h21_3", {0.932577,18.4514,1.0725}, {0,0,1.57});
    addObject("adream_door_h16", {-0.152311,22.3506,1.0725}, {0,0,3.14159});
    addObject("adream_door_h12", {-11.5018,11.6701,1.0725}, {0,0,-2.51327});
    addObject("adream_door_h14", {-6.56428,9.44934,1.0725}, {0,0,0});
    addObject("adream_door_h13", {-10.236,11.2895,1.0725}, {0,0,-0.942478});
    addObject("adream_door_h11", {-9.51151,14.0825,1.0725}, {0,0,-0.942478});

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
    Shape_t shape = PerceptionModuleBase_::getEntityShapeFromOntology(onto_, name);
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