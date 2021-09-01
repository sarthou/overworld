#ifndef OWDS_ROBOSHERLOCKPERCEPTIONMODULE_H
#define OWDS_ROBOSHERLOCKPERCEPTIONMODULE_H

#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/Agent.h"
#include "overworld/Perception/PerceptionModuleBase.h"

#include <visualization_msgs/MarkerArray.h>

namespace owds {

class RoboSherlockPerceptionModule : public PerceptionModuleRosBase<Object, visualization_msgs::MarkerArray>
{
public:
  RoboSherlockPerceptionModule(ros::NodeHandle* n, Agent* agent = nullptr);

  void setAgent(Agent* agent) { agent_ = agent; }

private:
  Agent* agent_;
  Pose last_head_pose_;

  bool perceptionCallback(const visualization_msgs::MarkerArray& markers);

  bool headHasMoved();

  void updateEntities(const visualization_msgs::MarkerArray& main_markers);
  bool createNewEntity(const visualization_msgs::Marker& marker);
};

} // namespace owds

#endif // OWDS_ROBOSHERLOCKPERCEPTIONMODULE_H