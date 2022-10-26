#include "overworld/Perception/Modules/ObjectsModules/FakeObjectPerceptionModule.h"

#include <pluginlib/class_list_macros.h>

namespace owds {

FakeObjectPerceptionModule::FakeObjectPerceptionModule() : PerceptionModuleRosBase("/overworld/fake_objects_poses"),
                                                            ontologies_manipulator_(nullptr),
                                                            onto_(nullptr),
                                                            tf2_listener_(tf_buffer_)
{}

bool FakeObjectPerceptionModule::closeInitialization()
{
  ontologies_manipulator_ = new OntologiesManipulator(n_);
  ontologies_manipulator_->waitInit();
  std::string robot_name = robot_agent_->getId();
  ontologies_manipulator_->add(robot_name);
  onto_ = ontologies_manipulator_->get(robot_name);
  onto_->close();

  return true;
}

bool FakeObjectPerceptionModule::perceptionCallback(const overworld::EntitiesPoses& msg)
{
  for(auto& percept : percepts_)
    percept.second.setUnseen();

  for(auto& obj : msg.entities)
  {
    auto it_percept = percepts_.find(obj.id);
    if(it_percept == percepts_.end())
      it_percept = percepts_.insert(std::make_pair(obj.id, createNewEntity(obj.id))).first;

    std::string frame_id = obj.pose.header.frame_id;
    if (frame_id[0] == '/')
        frame_id = frame_id.substr(1);
    geometry_msgs::TransformStamped to_map = tf_buffer_.lookupTransform("map", frame_id, ros::Time::now(), ros::Duration(1.0));
    geometry_msgs::PoseStamped obj_in_map;
    tf2::doTransform(obj.pose, obj_in_map, to_map);
    it_percept->second.updatePose(obj_in_map);
    it_percept->second.setSeen();
  }

  return true;
}

Object FakeObjectPerceptionModule::createNewEntity(const std::string& id)
{
  Object obj(id);

  Shape_t shape = PerceptionModuleBase_::getEntityShapeFromOntology(onto_, obj.id());
  if(shape.type == SHAPE_NONE)
  {
    shape.type = SHAPE_CUBE;
    shape.color = PerceptionModuleBase_::getEntityColorFromOntology(onto_, obj.id(), {0.8,0.8,0.8});
    shape.scale = {1, 1, 1};
  }
  obj.setShape(shape);
  obj.setMass(PerceptionModuleBase_::getEntityMassFromOntology(onto_, obj.id()));

  return obj;
}

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::FakeObjectPerceptionModule, owds::PerceptionModuleBase_<owds::Object>)