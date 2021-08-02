#include "overworld/Perception/Modalities/ArTrackPerceptionModule.h"

#include "overworld/Utility/ShellDisplay.h"

namespace owds {

#define TO_HALF_RAD M_PI / 180. / 2.

ArTrackPerceptionModule::ArTrackPerceptionModule(ros::NodeHandle* n, Agent* agent) : PerceptionModuleRosSyncBase(n, "ar_pose_marker", "ar_pose_visible_marker"),
                                                                                     ontologies_manipulator_(n),
                                                                                     tf2_listener_(tf_buffer_)
{
  agent_ = agent;

  ontologies_manipulator_.waitInit();
  ontologies_manipulator_.add("robot");
  onto_ = ontologies_manipulator_.get("robot");
  onto_->close();
}

bool ArTrackPerceptionModule::perceptionCallback(const ar_track_alvar_msgs::AlvarMarkers& markers,
                                                 const ar_track_alvar_msgs::AlvarVisibleMarkers& visible_markers)
{
  if(agent_ == nullptr)
    return false;
  else if(headHasMoved())
    return false;

  std::vector<ar_track_alvar_msgs::AlvarVisibleMarker> valid_visible_markers;
  std::unordered_set<size_t> invalid_main_markers_ids;
  for(const auto& visible_marker : visible_markers.markers)
  {
    if(isInValidArea(Pose({visible_marker.pose.pose.position.x,
                           visible_marker.pose.pose.position.y,
                           visible_marker.pose.pose.position.z},
                          {visible_marker.pose.pose.orientation.x,
                           visible_marker.pose.pose.orientation.y,
                           visible_marker.pose.pose.orientation.z,
                           visible_marker.pose.pose.orientation.w})))
        valid_visible_markers.push_back(visible_marker);
      else
        invalid_main_markers_ids.insert(visible_marker.main_id);
  }

  updateEntities(markers, invalid_main_markers_ids);
  setAllPoiUnseen();

  for(auto& visible_marker : valid_visible_markers)
    setPointOfInterest(visible_marker);

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

void ArTrackPerceptionModule::updateEntities(const ar_track_alvar_msgs::AlvarMarkers& main_markers,
                                             const std::unordered_set<size_t>& invalid_main_markers_ids)
{
  for(const auto& main_marker : main_markers.markers)
  {
    bool created = false;
    if(blacklist_ids_.find(main_marker.id) != blacklist_ids_.end())
      continue;
    else if(invalid_main_markers_ids.find(main_marker.id) != invalid_main_markers_ids.end())
      continue;
    else if(ids_map_.find(main_marker.id) == ids_map_.end())
    {
      createNewEntity(main_marker);
      created = true;
    }

    auto it_obj = percepts_.find(ids_map_[main_marker.id]);
    if(created || (main_marker.confidence < 0.2))
    {
      geometry_msgs::TransformStamped to_map = tf_buffer_.lookupTransform("map", main_marker.header.frame_id, main_marker.header.stamp, ros::Duration(1.0) );
      geometry_msgs::PoseStamped marker_in_map;
      tf2::doTransform(main_marker.pose, marker_in_map, to_map);
      it_obj->second.updatePose({marker_in_map.pose.position.x,
                                 marker_in_map.pose.position.y,
                                 marker_in_map.pose.position.z},
                                {marker_in_map.pose.orientation.x,
                                 marker_in_map.pose.orientation.y,
                                 marker_in_map.pose.orientation.z,
                                 marker_in_map.pose.orientation.w},
                                main_marker.header.stamp);
    }
  }
}

bool ArTrackPerceptionModule::createNewEntity(const ar_track_alvar_msgs::AlvarMarker& marker)
{
  auto true_id = onto_->individuals.getFrom("hasArId", "real#"+std::to_string(marker.id));
  if(true_id.size() == 0)
  {
    blacklist_ids_.insert(marker.id);
    return false;
  }

  ids_map_[marker.id] = true_id[0];
  Object obj(true_id[0]);

  Shape_t shape;
  auto meshs = onto_->individuals.getOn(obj.id(), "hasMesh");
  if(meshs.size())
  {
    shape.type = SHAPE_MESH;
    shape.mesh_resource = meshs[0].substr(meshs[0].find("#") + 1);

    if(onto_->individuals.isA(obj.id(), "Cube"))
      shape.color = {0,0,1};
    else if(onto_->individuals.isA(obj.id(), "Box"))
      shape.color = {0.82, 0.42, 0.12};
  }
  else
  {
    shape.type = SHAPE_CUBE;
    shape.color = {1,0,0};
    shape.scale = {0.05, 0.05, 0.003};
  }
  obj.setShape(shape);

  percepts_.insert(std::make_pair(obj.id(), obj));
}

} // namespace owds