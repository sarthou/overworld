#include "overworld/Perception/Modules/ObjectsModules/Pr2GripperPerceptionModule.h"

#include <numeric>

#include <pluginlib/class_list_macros.h>

#include "overworld/Utility/ShellDisplay.h"

namespace owds {

Pr2GripperPerceptionModule::Pr2GripperPerceptionModule() : PerceptionModuleRosBase("", true)
                                                           // We need to wait the initialization to know the topic name
{
  pr2_left_tip_bullet_id_ = -1;
  pr2_right_tip_bullet_id_ = -1;
  side_ = PR2_GRIPER_UNDEFINED;

  is_init_ = false;
  left_tip_pressure_ = 0;
  left_tip_pressure_prev_ = 0;
  right_tip_pressure_ = 0;
  right_tip_pressure_prev_ = 0;

  has_picked_ = false;
  obj_id_ = 0;

  pressure_threshold_ = 800;
  distance_threshold_ = 0.03;
  min_period_ = 0.5;
}

void Pr2GripperPerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
{
  if(parameter_name == "side")
    side_ = static_cast<Pr2GripperSide_e>(std::stoi(parameter_value));
  else if(parameter_name == "pressure_threshold")
    pressure_threshold_ = std::stod(parameter_value);
  else if(parameter_name == "distance_threshold")
    distance_threshold_ = std::stod(parameter_value);
  else if(parameter_name == "min_period")
    min_period_ = std::stod(parameter_value);
  else
      ShellDisplay::warning("[Pr2GripperPerceptionModule] Unkown parameter " + parameter_name);
}

bool Pr2GripperPerceptionModule::closeInitialization()
{
  if(side_ == PR2_GRIPER_UNDEFINED)
  {
    ShellDisplay::error("[Pr2GripperPerceptionModule] No gripper side has been defined");
    return false;
  }

  setTopicName("pressure/" + std::string((side_ == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_motor");

  auto p = bullet_client_->findJointAndLinkIndices(robot_bullet_id_);
  auto joint_name_id = p.first;
  auto link_name_id = p.second;

  std::string left_finger_link = std::string((side_ == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_l_finger_tip_link";
  std::string right_finger_link = std::string((side_ == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_r_finger_tip_link";

  if(link_name_id.find(left_finger_link) != link_name_id.end())
    pr2_left_tip_bullet_id_ = link_name_id.at(left_finger_link);
  else
    throw std::runtime_error("[Pr2GripperPerceptionModule] link " + left_finger_link + " does not exist");

  if(link_name_id.find(right_finger_link) != link_name_id.end())
    pr2_right_tip_bullet_id_ = link_name_id.at(right_finger_link);
  else
    throw std::runtime_error("[Pr2GripperPerceptionModule] link " + left_finger_link + " does not exist");

  return true;
}

bool Pr2GripperPerceptionModule::perceptionCallback(const pr2_msgs::PressureState& msg)
{
  if ((ros::Time::now() - last_update_).toSec() < min_period_)
        return false;
  last_update_ = ros::Time::now();

  left_tip_pressure_prev_ = left_tip_pressure_;
  right_tip_pressure_prev_ = right_tip_pressure_;

  left_tip_pressure_ = 0;
  for(auto& p : msg.l_finger_tip)
    left_tip_pressure_ += (double)p;
  left_tip_pressure_ /= (double)msg.l_finger_tip.size();

  right_tip_pressure_ = 0;
  for(auto& p : msg.r_finger_tip)
    right_tip_pressure_ += (double)p;
  right_tip_pressure_ /= (double)msg.r_finger_tip.size();

  if(is_init_)
  {
    double pressure_diff_left = left_tip_pressure_ - left_tip_pressure_prev_;
    double pressure_diff_right = right_tip_pressure_ - right_tip_pressure_prev_;

    if(has_picked_ == false)
    {
      if((pressure_diff_left > pressure_threshold_) && (pressure_diff_right > pressure_threshold_))
      {
        double dist = getGripperDistance();
        if(dist >= distance_threshold_)
        {
          if(side_ == PR2_GRIPPER_LEFT)
            current_obj_id_ = "obj_pr2_gripper_left_" + std::to_string(obj_id_);
          else
            current_obj_id_ = "obj_pr2_gripper_right_" + std::to_string(obj_id_);

          Object percept(current_obj_id_, false);
          Shape_t shape;
          shape.type = SHAPE_CUBE;
          shape.scale.fill(dist);
          percept.setShape(shape);
          // we can not take the mass from the ontology since the object does not exist in (fake id)
          percept.setSeen();

          percepts_.insert(std::make_pair(percept.id(), percept));

          if(side_ == PR2_GRIPPER_LEFT)
          {
            if(robot_agent_->getLeftHand() != nullptr)
            {
              percepts_.at(percept.id()).updatePose(robot_agent_->getLeftHand()->pose());
              percepts_.at(percept.id()).setInHand(robot_agent_->getLeftHand());
            }
          }
          else
          {
            if(robot_agent_->getRightHand() != nullptr)
            {
              percepts_.at(percept.id()).updatePose(robot_agent_->getRightHand()->pose());
              percepts_.at(percept.id()).setInHand(robot_agent_->getRightHand());
            }
          }

          has_picked_ = true;

          return true;
        }
      }

      return false;
    }
    else
    {
      if((pressure_diff_left < -pressure_threshold_) && (pressure_diff_right < -pressure_threshold_))
      {
        has_picked_ = false;
        // percepts_.at(current_obj_id_).unsetPose();
        percepts_.at(current_obj_id_).setUnseen();
        if(percepts_.at(current_obj_id_).isInHand())
          percepts_.at(current_obj_id_).removeFromHand();
        
        current_obj_id_ = "";
        obj_id_++;
      }
      else
      {
        if(side_ == PR2_GRIPPER_LEFT)
          {
            if(robot_agent_->getLeftHand() != nullptr)
              percepts_.at(percepts_.at(current_obj_id_).id()).updatePose(robot_agent_->getLeftHand()->pose());
          }
          else
          {
            if(robot_agent_->getRightHand() != nullptr)
              percepts_.at(percepts_.at(current_obj_id_).id()).updatePose(robot_agent_->getRightHand()->pose());
          }
      }
      
      return true;
    }

    return true;
  }
  else
  {
    is_init_ = true;
    return false;
  }
}

double Pr2GripperPerceptionModule::getGripperDistance()
{
  b3LinkState left_tip_link_state = bullet_client_->getLinkState(robot_bullet_id_, pr2_left_tip_bullet_id_);
  double* left_pos = left_tip_link_state.m_worldLinkFramePosition;
  double* left_rot = left_tip_link_state.m_worldLinkFrameOrientation;
  Pose left_tip_pose({left_pos[0], left_pos[1], left_pos[2]}, {left_rot[0], left_rot[1], left_rot[2], left_rot[3]});

  b3LinkState right_tip_link_state = bullet_client_->getLinkState(robot_bullet_id_, pr2_right_tip_bullet_id_);
  double* right_pos = right_tip_link_state.m_worldLinkFramePosition;
  double* right_rot = right_tip_link_state.m_worldLinkFrameOrientation;
  Pose right_tip_pose({right_pos[0], right_pos[1], right_pos[2]}, {right_rot[0], right_rot[1], right_rot[2], right_rot[3]});

  return left_tip_pose.distanceTo(right_tip_pose);
}

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::Pr2GripperPerceptionModule, owds::PerceptionModuleBase_<owds::Object>)