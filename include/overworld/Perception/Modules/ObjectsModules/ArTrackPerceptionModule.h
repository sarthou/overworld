#ifndef OWDS_ARTRACKPERCEPTIONMODULE_H
#define OWDS_ARTRACKPERCEPTIONMODULE_H

#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/Agent.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ar_track_alvar_msgs/AlvarVisibleMarkers.h"

#include "ontologenius/OntologiesManipulator.h"

#include <map>
#include <unordered_set>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace owds {

class ArTrackPerceptionModule : public PerceptionModuleRosSyncBase<Object, ar_track_alvar_msgs::AlvarMarkers, ar_track_alvar_msgs::AlvarVisibleMarkers>
{
public:
  ArTrackPerceptionModule(ros::NodeHandle* n, Agent* agent = nullptr);

  void setAgent(Agent* agent) { agent_ = agent; }

private:
  Agent* agent_;
  Pose last_head_pose_;

  std::map<size_t, std::string> ids_map_;
  std::unordered_set<size_t> blacklist_ids_;
  std::unordered_set<size_t> visible_markers_with_pois_;  // The id of the visible markers that we already seen at least once and were valid (and thus, have their pois created)

  OntologiesManipulator ontologies_manipulator_;
  OntologyManipulator* onto_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  bool perceptionCallback(const ar_track_alvar_msgs::AlvarMarkers& markers,
                          const ar_track_alvar_msgs::AlvarVisibleMarkers& visible_markers);

  bool headHasMoved();
  bool isInValidArea(const Pose& tag_pose);

  void setPointOfInterest(const ar_track_alvar_msgs::AlvarVisibleMarker& visible_marker);
  void setAllPoiUnseen();
  void updateEntities(const ar_track_alvar_msgs::AlvarMarkers& main_markers,
                      const std::unordered_set<size_t>& invalid_main_markers_ids);
  bool createNewEntity(const ar_track_alvar_msgs::AlvarMarker& marker);
  std::array<double, 3> getColor(const std::string& indiv_name, const std::array<double, 3>& default_value = {0,0,0});
};

} // namespace owds

#endif // OWDS_ARTRACKPERCEPTIONMODULE_H