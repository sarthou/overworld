#include "overworld/Perception/Modalities/ArTrackPerceptionModule.h"

#include "overworld/Utility/ShellDisplay.h"

namespace owds {

#define TO_HALF_RAD M_PI / 180. / 2.

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
  else if(agent_->getHead()->pose().angularDistance(last_head_pose_) > 0.001)
    res = true;
  
  last_head_pose_ = agent_->getHead()->pose();
  return res;
}

bool ArTrackPerceptionModule::isInValidArea(const Pose& tag_pose)
{
  auto tag_in_head = tag_pose.transformIn(agent_->getHead()->pose());
  if((tag_in_head.getZ() <= agent_->getFieldOfView().getClipFar()) &&
      (std::abs(tag_in_head.getOriginTilt()) <= agent_->getFieldOfView().getHeight() * TO_HALF_RAD) &&
      (std::abs(tag_in_head.getOriginPan()) < agent_->getFieldOfView().getWidth() * TO_HALF_RAD))
    return true;
  else
    return false;
}

void ArTrackPerceptionModule::setPointOfInterest(const ar_track_alvar_msgs::AlvarVisibleMarker& visible_marker)
{
  auto id_it = ids_map_.find(visible_marker.main_id);
  if(id_it == ids_map_.end())
  {
    ShellDisplay::warning("[ArTrackPerceptionModule] tag " + std::to_string(visible_marker.main_id) + " is unknown.");
    return;
  }

  std::string poi_id = "ar_" + std::to_string(visible_marker.id);
  auto obj_it = percepts_.find(id_it->second);

  for(const auto& poi : obj_it->second.getPointsOfInterest())
    if(poi.getId() == poi_id)
      return;

  double half_size = visible_marker.size / 100. / 2.; // we also put it in meters

  // TODO
}

void ArTrackPerceptionModule::setAllPoiUnseen()
{
  for(auto& percept : percepts_)
    percept.second.setAllPoiUnseen();
}

} // namespace owds