#include "overworld/Perception/Modalities/Pr2GripperPerceptionModule.h"

#include <numeric>

namespace owds {

Pr2GripperPerceptionModule::Pr2GripperPerceptionModule(ros::NodeHandle* n,
                                                       Pr2GripperSide_e side,
                                                       BulletClient* bullet_client,
                                                       int pr2_bullet_id,
                                                       Agent* agent) : PerceptionModuleRosBase(n, "pressure/" + std::string((side == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_motor")
{
  agent_ = agent;
  bullet_client_ = bullet_client;
  pr2_bullet_id_ = pr2_bullet_id;
  pr2_left_tip_bullet_id_ = -1;
  pr2_right_tip_bullet_id_ = -1;
  side_ = side;

  is_init_ = false;
  left_tip_pressure_ = 0;
  right_tip_pressure_ = 0;

  has_picked_ = false;
  obj_id_ = 0;

   auto p = bullet_client_->findJointAndLinkIndices(pr2_bullet_id_);
  auto joint_name_id = p.first;
  auto link_name_id = p.second;

  std::string left_finger_link = std::string((side == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_l_finger_tip_link";
  std::string right_finger_link = std::string((side == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_r_finger_tip_link";

  if(link_name_id.find(left_finger_link) != link_name_id.end())
    pr2_left_tip_bullet_id_ = link_name_id.at(left_finger_link);
  else
    throw std::runtime_error("[Pr2GripperPerceptionModule] link " + left_finger_link + " does not exist");

  if(link_name_id.find(right_finger_link) != link_name_id.end())
    pr2_right_tip_bullet_id_ = link_name_id.at(right_finger_link);
  else
    throw std::runtime_error("[Pr2GripperPerceptionModule] link " + left_finger_link + " does not exist");
}

bool Pr2GripperPerceptionModule::perceptionCallback(const pr2_msgs::PressureState& msg)
{
  left_tip_pressure_prev_ = left_tip_pressure_;
  right_tip_pressure_prev_ = right_tip_pressure_;

  left_tip_pressure_ = std::accumulate(msg.l_finger_tip.begin(), msg.l_finger_tip.end(), decltype(msg.l_finger_tip)::value_type(0));
  right_tip_pressure_ = std::accumulate(msg.r_finger_tip.begin(), msg.r_finger_tip.end(), decltype(msg.r_finger_tip)::value_type(0));

  if(is_init_)
  {
    double pressure_diff_left = left_tip_pressure_ - left_tip_pressure_prev_;
    double pressure_diff_right = right_tip_pressure_ - right_tip_pressure_prev_;

    if(has_picked_ == false)
    {
      if((pressure_diff_left > 4000) && (pressure_diff_right > 4000))
      {
        double dist = getGripperDistance();
        if(dist >= 0.04) // 4cm
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

          percepts_.insert(std::make_pair(percept.id(), percept));

          if(side_ == PR2_GRIPPER_LEFT)
          {
            if(agent_->getLeftHand() != nullptr)
              percepts_.at(percept.id()).setInHand(agent_->getLeftHand());
          }
          else
          {
            if(agent_->getRightHand() != nullptr)
              percepts_.at(percept.id()).setInHand(agent_->getRightHand());
          }

          return true;
        }
      }

      return false;
    }
    else
    {
      if((pressure_diff_left < 4000) && (pressure_diff_right < 4000))
      {
        has_picked_ = false;
        percepts_.at(current_obj_id_).unsetPose();
        if(percepts_.at(current_obj_id_).isInHand())
          percepts_.at(current_obj_id_).removeFromHand();
        
        current_obj_id_ = "";
        obj_id_++;
        return true;
      }
      else
        return false;
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
  b3LinkState left_tip_link_state = bullet_client_->getLinkState(pr2_bullet_id_, pr2_left_tip_bullet_id_);
  double* left_pos = left_tip_link_state.m_worldLinkFramePosition;
  double* left_rot = left_tip_link_state.m_worldLinkFrameOrientation;
  Pose left_tip_pose({left_pos[0], left_pos[1], left_pos[2]}, {left_rot[0], left_rot[1], left_rot[2], left_rot[3]});

  b3LinkState right_tip_link_state = bullet_client_->getLinkState(pr2_bullet_id_, pr2_right_tip_bullet_id_);
  double* right_pos = right_tip_link_state.m_worldLinkFramePosition;
  double* right_rot = right_tip_link_state.m_worldLinkFrameOrientation;
  Pose right_tip_pose({right_pos[0], right_pos[1], right_pos[2]}, {right_rot[0], right_rot[1], right_rot[2], right_rot[3]});

  return left_tip_pose.distanceSqTo(right_tip_pose);
}

} // namespace owds