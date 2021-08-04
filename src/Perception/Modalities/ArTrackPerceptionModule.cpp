#include "overworld/Perception/Modalities/ArTrackPerceptionModule.h"

#include "overworld/Utility/ShellDisplay.h"
#include <unordered_map>

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
  std::unordered_map<size_t, double> main_marker_id_max_confidence;
  for (const auto& visible_marker : visible_markers.markers)
  {
      geometry_msgs::PoseStamped marker_pose;
      auto old_pose = visible_marker.pose;
      if (old_pose.header.frame_id[0] == '/')
          old_pose.header.frame_id = old_pose.header.frame_id.substr(1);
      tf_buffer_.transform(old_pose, marker_pose, "map", ros::Duration(1.0));
      if (isInValidArea(Pose(marker_pose)) && visible_marker.confidence < 0.2)
          valid_visible_markers.push_back(visible_marker);
      else
          invalid_main_markers_ids.insert(visible_marker.main_id);
  }

  updateEntities(markers, invalid_main_markers_ids);
  setAllPoiUnseen();

  for (auto& visible_marker : valid_visible_markers)
  {
      if (visible_markers_with_pois_.count(visible_marker.id) == 0 && percepts_.count(ids_map_[visible_marker.main_id]) != 0)
      {
          // This visible marker has never been seen before (or was not valid) or its entity was not created, let's create its pois
          setPointOfInterest(visible_marker);
          visible_markers_with_pois_.insert(visible_marker.id);
      }
  }

  for (const auto& seen_visible_markers : visible_markers.markers)
  {
      // For all the seen marker (even the invalid) if the entity has been created, we said it has been seen
      if (percepts_.count(ids_map_[seen_visible_markers.main_id]) != 0)
      {
          percepts_.at(ids_map_[seen_visible_markers.main_id]).setSeen();
      }
  }
  return true;
}

bool ArTrackPerceptionModule::headHasMoved()
{
  bool res = false;

  if(agent_->getHead() == nullptr)
    res = true;
  else if(agent_->getHead()->isLocated() == false)
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
  return agent_->getFieldOfView().hasIn(tag_in_head);
}

void ArTrackPerceptionModule::setPointOfInterest(const ar_track_alvar_msgs::AlvarVisibleMarker& visible_marker)
{
    auto id_it = ids_map_.find(visible_marker.main_id);
    if (id_it == ids_map_.end())
    {
        ShellDisplay::warning("[ArTrackPerceptionModule] tag " + std::to_string(visible_marker.main_id) + " is unknown.");
        return;
    }

    std::string poi_id = "ar_" + std::to_string(visible_marker.id);
    auto obj_it = percepts_.find(id_it->second);

    if(obj_it->second.isLocated() == false)
      return;

    for (const auto& poi : obj_it->second.getPointsOfInterest())
        if (poi.getId() == poi_id)
            return;

    double half_size = visible_marker.size / 100. / 2.; // we also put it in meters

    PointOfInterest p(poi_id);
    Pose sub_pois[5] = {Pose({{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}), 
                        Pose({{-half_size, -half_size, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}),
                        Pose({{half_size, -half_size, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}), 
                        Pose({{half_size, half_size, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}),
                        Pose({{-half_size, half_size, 0.0}}, {{0.0, 0.0, 0.0, 1.0}})};
    geometry_msgs::PoseStamped map_to_visible_marker_g;
    auto old_pose = visible_marker.pose;
    if(old_pose.header.frame_id[0] == '/')
      old_pose.header.frame_id = old_pose.header.frame_id.substr(1);
    tf_buffer_.transform(old_pose, map_to_visible_marker_g, "map", ros::Duration(1.0));
    Pose map_to_visible_marker(map_to_visible_marker_g);
    Pose map_to_marked_object = obj_it->second.pose();

    Pose marker_in_marked_obj = map_to_visible_marker.transformIn(map_to_marked_object);
    for (const auto& sub_poi : sub_pois)
    {
      Pose marked_obj_to_poi = marker_in_marked_obj * sub_poi;
      p.addPoint(marked_obj_to_poi);
    }
    obj_it->second.addPointOfInterest(p);
}

void ArTrackPerceptionModule::setAllPoiUnseen()
{
  for(auto& percept : percepts_)
  {
    percept.second.setAllPoiUnseen();
    percept.second.setUnseen();
  }
}

void ArTrackPerceptionModule::updateEntities(const ar_track_alvar_msgs::AlvarMarkers& main_markers,
                                             const std::unordered_set<size_t>& invalid_main_markers_ids)
{
  for(const auto& main_marker : main_markers.markers)
  {
      bool created = false;
      if (blacklist_ids_.find(main_marker.id) != blacklist_ids_.end())
          continue;
      else if (invalid_main_markers_ids.find(main_marker.id) != invalid_main_markers_ids.end())
      {
          continue;
      }
      else if (ids_map_.find(main_marker.id) == ids_map_.end())
      {
          createNewEntity(main_marker);
      }

      auto it_obj = percepts_.find(ids_map_[main_marker.id]);
      std::string frame_id = main_marker.header.frame_id;
      if (frame_id[0] == '/')
          frame_id = frame_id.substr(1);
      geometry_msgs::TransformStamped to_map = tf_buffer_.lookupTransform("map", frame_id, main_marker.header.stamp, ros::Duration(1.0));
      geometry_msgs::PoseStamped marker_in_map;
      tf2::doTransform(main_marker.pose, marker_in_map, to_map);
      it_obj->second.updatePose(marker_in_map);
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