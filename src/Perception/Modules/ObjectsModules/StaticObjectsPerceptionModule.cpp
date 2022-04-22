#include "overworld/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.h"

#include <pluginlib/class_list_macros.h>

namespace owds {

StaticObjectsPerceptionModule::StaticObjectsPerceptionModule() : onto_(nullptr)
{
    addObject("walls", "package://overworld/models/obj/walls.obj",
                "package://overworld/models/obj/walls_vhacd.obj",
               {0,0,0}, {1.57,0,0}, {0.8,0.8,0.8});
    addObject("floor", "package://overworld/models/obj/adream_floor.obj", "",
               {0,0,0}, {1.57,0,0}, {0.8,0.8,0.8});
    addObject("glass_h10", "package://overworld/models/obj/glass_wall_h10.obj", "",
               {-5.101,20.266,1.444}, {1.5708,0,0.6379230776}, {0,0,0});
    addObject("glass_h20_1", "package://overworld/models/obj/glass_wall_h20_1.obj", "",
               {11.573,6.037,1.589}, {1.5708,0,0}, {0,0,0});
    addObject("glass_h20_2", "package://overworld/models/obj/glass_wall_h20_2.obj", "",
               {-4.052,8.304,1.426}, {1.57,0,0}, {0,0,0});
    addObject("window_h20", "package://overworld/models/obj/window_h20.obj", "",
               {-0.064,3.914,1.703}, {1.57,0,0}, {0,0,0});
    addObject("appartment", "package://overworld/models/obj/appartment.obj",
                "package://overworld/models/obj/appartment_vhacd.obj",
               {5.467,10.399,1.448}, {1.57,0,0}, {1,1,1});
}

bool StaticObjectsPerceptionModule::closeInitialization()
{
  ontologies_manipulator_ = new OntologiesManipulator(n_);
  ontologies_manipulator_->waitInit();
  std::string robot_name = robot_agent_->getId();
  ontologies_manipulator_->add(robot_name);
  onto_ = ontologies_manipulator_->get(robot_name);
  onto_->close();

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
    Pose pose(translation, rotation); 
    obj.updatePose(pose);
    obj.setStatic();
    updated_ = true;

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

    ShellDisplay::success("[StaticObjectsPerceptionModule] create shape for " + name);

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
    }
    else
        ShellDisplay::warning("[StaticObjectsPerceptionModule] No mesh defined in the ontology for entity \'" + name + "\'");    
}

} //namespace owds

PLUGINLIB_EXPORT_CLASS(owds::StaticObjectsPerceptionModule, owds::PerceptionModuleBase_<owds::Object>)