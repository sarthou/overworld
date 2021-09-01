#include "overworld/Perception/Modalities/RoboSherlockPerceptionModule.h"

#include "overworld/Utility/ShellDisplay.h"
#include <unordered_map>

namespace owds {

#define TO_HALF_RAD M_PI / 180. / 2.

RoboSherlockPerceptionModule::RoboSherlockPerceptionModule(ros::NodeHandle* n, Agent* agent) : PerceptionModuleRosBase(n, "robosherlock_marker", true)
{
  agent_ = agent;
}

bool RoboSherlockPerceptionModule::perceptionCallback(const visualization_msgs::MarkerArray& markers)
{
  if(agent_ == nullptr)
    return false;
  else if(headHasMoved())
    return false;

  updateEntities(markers);

  return true;
}

bool RoboSherlockPerceptionModule::headHasMoved()
{
    if (agent_->getHead() == nullptr)
        return true;
    if (agent_->getHead()->isLocated() == false)
        return true;
    return agent_->getHead()->hasMoved();
}

void RoboSherlockPerceptionModule::updateEntities(const visualization_msgs::MarkerArray& markers)
{
  for(auto percept : percepts_)
    percept.second.setUnseen();

  geometry_msgs::PoseStamped marker_pose;
  for(const auto& marker : markers.markers)
  {
      if(createNewEntity(marker) == false)
        continue;
  
      auto it_obj = percepts_.find("rs_" + std::to_string(marker.id));
      marker_pose.pose = marker.pose;
      marker_pose.header = marker.header;
      it_obj->second.updatePose(marker_pose);
      it_obj->second.setSeen();  
  }
}

bool RoboSherlockPerceptionModule::createNewEntity(const visualization_msgs::Marker& marker)
{
  Object obj("rs_" + std::to_string(marker.id), false);

  Shape_t shape;

  shape.type = (ShapeType_e) marker.type;
  shape.scale[0] = (double) marker.scale.x;
  shape.scale[1] = (double) marker.scale.y;
  shape.scale[2] = (double) marker.scale.z;

  shape.color[0] = marker.color.r;
  shape.color[1] = marker.color.g;
  shape.color[2] = marker.color.b;
 
  obj.setShape(shape);

  percepts_.insert(std::make_pair(obj.id(), obj));

  return true;
}

} // namespace owds