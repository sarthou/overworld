#include "overworld/Perception/Modules/RobotsModules/FakeRobotPerceptionModule.h"

#include <ros/package.h>

#include <pluginlib/class_list_macros.h>

namespace owds {

FakeRobotPerceptionModule::FakeRobotPerceptionModule(): PerceptionModuleRosBase("/overworld/fake_robot_poses"),
                                                        base_link_("base_footprint"),
                                                        ontologies_manipulator_(nullptr),
                                                        onto_(nullptr),
                                                        tf2_listener_(tf_buffer_)
{}

void FakeRobotPerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
{
    if(parameter_name == "name")
        robot_name_ = parameter_value;
    else if(parameter_name == "head")
        head_link_ = parameter_value;
    else if(parameter_name == "base")
        base_link_ = parameter_value;
    else if(parameter_name == "head_pose")
        head_pose_ = stringToPose(parameter_value);
    else if(parameter_name == "base_pose")
        base_pose_ = stringToPose(parameter_value);
    else
        ShellDisplay::warning("[FakeRobotPerceptionModule] Unkown parameter " + parameter_name);
}

bool FakeRobotPerceptionModule::closeInitialization()
{
    if(robot_name_ == "")
    {
        ShellDisplay::error("[FakeRobotPerceptionModule] No robot name has been defined");
        return false;
    }
    if(head_link_ == "")
    {
        ShellDisplay::error("[FakeRobotPerceptionModule] No head link has been defined");
        return false;
    }

    ontologies_manipulator_ = new OntologiesManipulator(n_);
    ontologies_manipulator_->waitInit();
    ontologies_manipulator_->add(robot_name_);
    onto_ = ontologies_manipulator_->get(robot_name_);
    onto_->close();

    BodyPart head = createBodyPart(robot_name_, head_link_, BodyPartType_e::BODY_PART_HEAD);
    head.updatePose(head_pose_);
    percepts_.insert(std::make_pair(head_link_, head));
    BodyPart base = createBodyPart(robot_name_, base_link_, BodyPartType_e::BODY_PART_BASE);
    base.updatePose(base_pose_);
    percepts_.insert(std::make_pair(base_link_, base));

    return true;
}

bool FakeRobotPerceptionModule::perceptionCallback(const overworld::AgentPose& msg)
{
    if (msg.parts.size() == 0)
        return false;

    for(auto& part : msg.parts)
    {
        auto it_percept = percepts_.find(part.id);
        if(it_percept == percepts_.end())
            continue;

        std::string frame_id = part.pose.header.frame_id;
        if (frame_id[0] == '/')
            frame_id = frame_id.substr(1);

        try {
            geometry_msgs::TransformStamped to_map = tf_buffer_.lookupTransform("map", frame_id, part.pose.header.stamp, ros::Duration(1.0));
            geometry_msgs::PoseStamped part_in_map;
            tf2::doTransform(part.pose, part_in_map, to_map);
            it_percept->second.updatePose(part_in_map);
        }
        catch (const tf2::TransformException& ex) {
            ShellDisplay::error("[FakeHumanPerceptionModule]" + std::string(ex.what()));
        }
    }
    
  return true;
}

BodyPart FakeRobotPerceptionModule::createBodyPart(const std::string& robot_name, const std::string& part_name, BodyPartType_e part_type)
{
  BodyPart part(part_name);
  part.setAgentName(robot_name);
  part.setType(part_type);

  Shape_t shape = PerceptionModuleBase_::getEntityShapeFromOntology(onto_, part_name);
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

      shape.color = PerceptionModuleBase_::getEntityColorFromOntology(onto_, part_name, {0.7, 0.7, 0.7});
  }
  part.setShape(shape);

  return part;
}

Pose FakeRobotPerceptionModule::stringToPose(const std::string& str)
{
  size_t start;
  size_t end = 0;
  std::vector<double> out;

  while ((start = str.find_first_not_of(" ", end)) != std::string::npos)
  {
    end = str.find(" ", start);
    out.push_back(std::atof(str.substr(start, end - start).c_str()));
  }
  if(out.size() == 7)
    return Pose({out[0], out[1], out[2]}, {out[3], out[4], out[5], out[6]});
  else
  {
    ShellDisplay::error("[FakeRobotPerceptionModule] position \"" + str + "\" is malformed. Expected format is \"x y z rx ry rz rw\"");
    return Pose();
  }
}

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::FakeRobotPerceptionModule, owds::PerceptionModuleBase_<owds::BodyPart>)