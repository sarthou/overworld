#include "overworld/Perception/Modules/ObjectsModules/FakeObjectPerceptionModule.h"

#include <pluginlib/class_list_macros.h>

namespace owds {

  FakeObjectPerceptionModule::FakeObjectPerceptionModule() : PerceptionModuleRosBase(""),
                                                             ontologies_manipulator_(nullptr),
                                                             onto_(nullptr),
                                                             tf2_listener_(tf_buffer_),
                                                             topic_name_("/overworld/fake_objects_poses"),
                                                             true_id_(true)
  {}

  bool FakeObjectPerceptionModule::closeInitialization()
  {
    setTopicName(topic_name_);

    ontologies_manipulator_ = new onto::OntologiesManipulator();
    ontologies_manipulator_->waitInit();
    std::string robot_name = robot_agent_->getId();
    ontologies_manipulator_->add(robot_name);
    onto_ = ontologies_manipulator_->get(robot_name);
    onto_->close();

    return true;
  }

  void FakeObjectPerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
  {
    if(parameter_name == "true_id")
      true_id_ = (parameter_value == "true");
    else if(parameter_name == "topic")
      topic_name_ = parameter_value;
    else if(parameter_name == "sensor_id")
      sensor_id_ = parameter_value;
    else
      ShellDisplay::warning("[StaticObjectsPerceptionModule] Unknown parameter " + parameter_name);
  }

  bool FakeObjectPerceptionModule::perceptionCallback(const overworld::EntitiesPoses& msg)
  {
    for(auto& percept : percepts_)
      percept.second.setUnseen();

    for(auto& obj : msg.entities)
    {
      auto it_percept = percepts_.find(obj.id);
      if(it_percept == percepts_.end())
      {
        it_percept = percepts_.insert(std::make_pair(obj.id, createPercept(obj.id))).first;
        it_percept->second.setSensorId(sensor_id_.empty() ? robot_agent_->getHead()->id() : sensor_id_);
      }

      std::string frame_id = obj.pose.header.frame_id;
      if(frame_id[0] == '/')
        frame_id = frame_id.substr(1);

      try
      {
        geometry_msgs::TransformStamped to_map = tf_buffer_.lookupTransform("map", frame_id, ros::Time::now(), ros::Duration(1.0));
        geometry_msgs::PoseStamped obj_in_map;
        tf2::doTransform(obj.pose, obj_in_map, to_map);
        it_percept->second.updatePose(obj_in_map);
        it_percept->second.setSeen();
      }
      catch(const tf2::TransformException& ex)
      {
        ShellDisplay::error("[FakeObjectPerceptionModule]" + std::string(ex.what()));
        return false;
      }
    }

    return true;
  }

  Percept<Object> FakeObjectPerceptionModule::createPercept(const std::string& id)
  {
    Percept<Object> obj(id, true_id_);

    Shape_t shape = ontology::getEntityShape(onto_, obj.id());
    if(shape.type == SHAPE_NONE)
    {
      shape.type = SHAPE_CUBE;
      shape.color = ontology::getEntityColor(onto_, obj.id(), {0.8, 0.8, 0.8});
      shape.scale = {0.05, 0.05, 0.05};
    }
    obj.setShape(shape);
    obj.setMass(ontology::getEntityMass(onto_, obj.id()));

    return obj;
  }

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::FakeObjectPerceptionModule, owds::PerceptionModuleBase_<owds::Object>)