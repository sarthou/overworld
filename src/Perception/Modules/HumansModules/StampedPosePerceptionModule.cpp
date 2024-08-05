#include "overworld/Perception/Modules/HumansModules/StampedPosePerceptionModule.h"

#include <pluginlib/class_list_macros.h>

#include "overworld/Utility/ShellDisplay.h"

namespace owds {

  StampedPosePerceptionModule::StampedPosePerceptionModule() : PerceptionModuleRosBase("/peopleTrack"),
                                                               ontologies_manipulator_(nullptr),
                                                               onto_(nullptr)
  {}

  void StampedPosePerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
  {
    if(parameter_name == "name")
      human_name_ = parameter_value;
    else
      ShellDisplay::warning("[StampedPosePerceptionModule] Unkown parameter " + parameter_name);
  }

  bool StampedPosePerceptionModule::closeInitialization()
  {
    if(human_name_ == "")
    {
      ShellDisplay::error("[StampedPosePerceptionModule] No human name has been defined");
      return false;
    }

    ontologies_manipulator_ = new onto::OntologiesManipulator();
    ontologies_manipulator_->waitInit();
    ontologies_manipulator_->add(robot_agent_->getId());
    onto_ = ontologies_manipulator_->get(robot_agent_->getId());
    onto_->close();

    auto head_names = onto_->individuals.getOn(human_name_, "hasHead");
    if(head_names.size() == 0)
    {
      ShellDisplay::error("No head defined in the ontology for human: '" + human_name_ + "'.");
      throw std::runtime_error("No head defined in the ontology for human: '" + human_name_ + "'.");
    }

    head_name_ = head_names.at(0);

    std::array<double, 3> default_color = {0.976470588, 0.894117647, 0.717647059};
    ;

    percepts_.emplace(head_name_, head_name_);
    percepts_.at(head_name_).setAgentName(human_name_);
    percepts_.at(head_name_).setType(BodyPartType_e::BODY_PART_HEAD);
    Shape_t head_shape = ontology::getEntityShape(onto_, head_name_);
    if(head_shape.type == SHAPE_NONE)
    {
      head_shape.type = SHAPE_SPEHERE;
      head_shape.scale = {0.12, 0.15, 0.2};
      head_shape.color = ontology::getEntityColor(onto_, head_name_, default_color);
    }
    percepts_.at(head_name_).setShape(head_shape);

    return true;
  }

  bool StampedPosePerceptionModule::perceptionCallback(const geometry_msgs::PoseStamped& msg)
  {
    Pose p({msg.pose.position.x, msg.pose.position.y, msg.pose.position.z},
           {msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w});

    ros::Time stamp(msg.header.stamp);
    percepts_.at(head_name_).updatePose(p, stamp);

    return true;
  }

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::StampedPosePerceptionModule, owds::PerceptionModuleBase_<owds::BodyPart>)
