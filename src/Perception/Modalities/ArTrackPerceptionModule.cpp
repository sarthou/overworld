#include "overworld/Perception/Modalities/ArTrackPerceptionModule.h"

namespace owds {

ArTrackPerceptionModule::ArTrackPerceptionModule(ros::NodeHandle* n, Agent* agent) : PerceptionModuleRosSyncBase(n, "ar_pose_marker", "ar_pose_visible_marker")
{
  agent_ = agent;
}

bool ArTrackPerceptionModule::perceptionCallback(const ar_track_alvar_msgs::AlvarMarkers& markers,
                                                 const ar_track_alvar_msgs::AlvarVisibleMarkers& visible_markers)
{
  if(agent_ == nullptr)
    return false;
  else if(headHasMoved())
    return false;

  return true;
}

bool ArTrackPerceptionModule::headHasMoved()
{
  bool res = false;

  if(agent_->getHead()->isLocated() == false)
    res = true;
  else if(agent_->getHead()->pose().distanceTo(last_head_pose_) > 0.001)
    res = true;
  else if(agent_->getHead()->pose().angleTo(last_head_pose_) > 0.001)
    res = true;
  
  last_head_pose_ = agent_->getHead()->pose();
  return res;
}

bool ArTrackPerceptionModule::isInValidArea(const Pose& tag_pose)
{
  auto tag_in_head = tag_pose.transform(agent_->getHead()->pose());
  return true;
}

} // namespace owds