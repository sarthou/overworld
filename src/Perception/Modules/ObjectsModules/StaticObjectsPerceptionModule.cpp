#include "overworld/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.h"

namespace owds {

StaticObjectsPerceptionModule::StaticObjectsPerceptionModule()
{
    addObject("walls", "package://overworld/models/obj/walls.obj",
               {0,0,0}, {1.57,0,0}, {0.8,0.8,0.8});
    addObject("floor", "package://overworld/models/obj/adream_floor.obj",
               {0,0,0}, {1.57,0,0}, {0.8,0.8,0.8});
    addObject("glass_h10", "package://overworld/models/obj/glass_wall_h10.obj",
               {-5.101,20.266,1.444}, {1.5708,0,0.6379230776}, {0,0,0});
    addObject("glass_h20_1", "package://overworld/models/obj/glass_wall_h20_1.obj",
               {11.573,6.037,1.589}, {1.5708,0,0}, {0,0,0});
    addObject("glass_h20_2", "package://overworld/models/obj/glass_wall_h20_2.obj",
               {-4.052,8.304,1.426}, {1.57,0,0}, {0,0,0});
    addObject("window_h20", "package://overworld/models/obj/window_h20.obj",
               {-0.064,3.914,1.703}, {1.57,0,0}, {0,0,0});
    addObject("appartment", "package://overworld/models/obj/appartment.obj",
               {5.467,10.399,1.448}, {1.57,0,0}, {1,1,1}); 
}

void StaticObjectsPerceptionModule::addObject(const std::string& name,
                                              const std::string& file,
                                              const std::array<double,3>& translation,
                                              const std::array<double,3>& rotation,
                                              const std::array<double,3>& color)
{
    Object obj(name);
    Shape_t shape;
    shape.type = SHAPE_MESH;
    shape.mesh_resource = file;
    shape.color = color;
    obj.setShape(shape);
    Pose pose(translation, rotation); 
    obj.updatePose(pose);
    obj.setStatic();
    updated_ = true;

    percepts_.insert(std::make_pair(obj.id(), obj));
}

} //namespace owds