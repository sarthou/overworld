#include "overworld/Perception/Modules/HumansModules/FakeHumansPerceptionModule.h"

#include <pluginlib/class_list_macros.h>

#include "overworld/Utils/ShellDisplay.h"

namespace owds {

  FakeHumansPerceptionModule::FakeHumansPerceptionModule() : PerceptionModuleRosBase("/overworld/fake_humans_poses"),
                                                             ontologies_manipulator_(nullptr),
                                                             onto_(nullptr),
                                                             tf2_listener_(tf_buffer_)
  {}

  bool FakeHumansPerceptionModule::closeInitialization()
  {
    ontologies_manipulator_ = new onto::OntologiesManipulator();
    ontologies_manipulator_->waitInit();
    ontologies_manipulator_->add(robot_agent_->getId());
    onto_ = ontologies_manipulator_->get(robot_agent_->getId());
    onto_->close();

    return true;
  }

  bool FakeHumansPerceptionModule::perceptionCallback(const overworld::AgentsPose& msg)
  {
    for(auto& percept : percepts_) // TODO add the sensors
      percept.second.setSensorId(robot_agent_->getHead()->id());
    // An agent can have several sensors, which one do we set if more than one sensor sees the percept?

    if(msg.agents.size() == 0)
      return false;
    for(auto& agent : msg.agents)
      for(auto& part : agent.parts)
      {
        auto it_percept = percepts_.find(part.id);
        if(it_percept == percepts_.end())
          it_percept = percepts_.insert(std::make_pair(part.id, createPercept(agent.id, part.id))).first;

        std::string frame_id = part.pose.header.frame_id;
        if(frame_id[0] == '/')
          frame_id = frame_id.substr(1);

        try
        {
          geometry_msgs::PoseStamped part_in_map;
          if(part.pose.header.frame_id != "/map")
          {
            geometry_msgs::TransformStamped to_map = tf_buffer_.lookupTransform("map", frame_id, part.pose.header.stamp, ros::Duration(1.0));
            tf2::doTransform(part.pose, part_in_map, to_map);
          }
          else
            part_in_map = part.pose;
          it_percept->second.updatePose(part_in_map);
        }
        catch(const tf2::TransformException& ex)
        {
          ShellDisplay::error("[FakeHumanPerceptionModule]" + std::string(ex.what()));
          return false;
        }
      }

    return true;
  }

  Percept<BodyPart> FakeHumansPerceptionModule::createPercept(const std::string& human_name, const std::string& part_name)
  {
    BodyPartType_e part_type = BodyPartType_e::BODY_PART_UNKNOW;

    auto types = onto_->individuals.getUp(part_name);
    if(std::find(types.begin(), types.end(), "Head") != types.end())
      part_type = BodyPartType_e::BODY_PART_HEAD;
    else if(std::find(types.begin(), types.end(), "LeftHand") != types.end())
      part_type = BodyPartType_e::BODY_PART_LEFT_HAND;
    else if(std::find(types.begin(), types.end(), "RightHand") != types.end())
      part_type = BodyPartType_e::BODY_PART_RIGHT_HAND;
    else if(std::find(types.begin(), types.end(), "Base") != types.end())
      part_type = BodyPartType_e::BODY_PART_BASE;
    else if(std::find(types.begin(), types.end(), "Torso") != types.end())
      part_type = BodyPartType_e::BODY_PART_TORSO;

    Percept<BodyPart> part(part_name);
    part.setAgentName(human_name);
    part.setType(part_type);

    Shape_t shape = ontology::getEntityShape(onto_, part_name);
    if(shape.type == SHAPE_NONE)
    {
      if(part_type == BodyPartType_e::BODY_PART_HEAD)
      {
        shape.type = SHAPE_SPEHERE;
        shape.scale = {0.12, 0.15, 0.2};
      }
      else if((part_type == BodyPartType_e::BODY_PART_LEFT_HAND) || (part_type == BodyPartType_e::BODY_PART_RIGHT_HAND))
      {
        shape.type = SHAPE_CUBE;
        shape.scale = {0.10, 0.03, 0.18};
      }
      else if((part_type == BodyPartType_e::BODY_PART_TORSO) || (part_type == BodyPartType_e::BODY_PART_BASE))
      {
        shape.type = SHAPE_CUBE;
        shape.scale = {0.5, 0.2, 0.85};
      }

      shape.color = ontology::getEntityColor(onto_, part_name, {0.976470588, 0.894117647, 0.717647059});
    }
    part.setShape(shape);

    return part;
  }

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::FakeHumansPerceptionModule, owds::PerceptionModuleBase_<owds::BodyPart>)
