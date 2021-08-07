#include "overworld/Perception/ObjectsPerceptionManager.h"

#include "overworld/BasicTypes/Hand.h"

#define TO_HALF_RAD M_PI / 180. / 2.

namespace owds {

void ObjectsPerceptionManager::getPercepts(const std::map<std::string, Object>& percepts)
{
  for(auto& percept : percepts)
    {
        auto it = entities_.find(percept.second.id());
        if(it == entities_.end())
        {
            if(percept.second.isLocated() == false)
              continue;

            auto new_object = new Object(percept.second);
            it = entities_.insert(std::pair<std::string, Object*>(percept.second.id(), new_object)).first;
            addToBullet(it->second);
        }
        
        if(it->second->isInHand())
        {
          if(lost_objects_nb_frames_.find(percept.first) != lost_objects_nb_frames_.end())
            if(lost_objects_nb_frames_[percept.first] < 5)
            {
              if(it->second->pose().distanceSqTo(percept.second.pose()) > 0.55) // Add a condition on time ?
                it->second->removeFromHand();
              else
                updateEntityPose(it->second, it->second->getHandIn()->pose(), ros::Time::now());
            }
        }
        else if (percept.second.hasBeenSeen())
          updateEntityPose(it->second, percept.second.pose(), percept.second.lastStamp());
    }
}

void ObjectsPerceptionManager::reasoningOnUpdate()
{
  std::vector<Object*> no_data_objects;
  bullet_client_->performCollisionDetection();
  for(auto& object : entities_)
  {
    if(object.second->isStatic() == true)
      continue;
    else if (object.second->isLocated() == false)
      continue;
    

    if(object.second->isInHand() == false)
    {
      if(object.second->getNbFrameUnseen() >= 5)
      {
        if(object.second->getPointsOfInterest().size() != 0)
        {
          auto pois_in_fov = getPoisInFov(object.second);
          if(pois_in_fov.size() != 0)
          {
            if(shouldBeSeen(object.second, pois_in_fov))
            {
              auto it_unseen = lost_objects_nb_frames_.find(object.first);
              if(it_unseen == lost_objects_nb_frames_.end())
                it_unseen = lost_objects_nb_frames_.insert(std::make_pair<std::string, size_t>(object.second->id(), size_t(0))).first;
              it_unseen->second++;
              if(it_unseen->second > 5)
              {
                removeEntityPose(object.second);
              }
            }
          }
        }
        else
        {
          // Object has no poi
          no_data_objects.push_back(object.second);
        }
      }
    }
  }

  if(no_data_objects.size())
  {
    auto objects_in_camera = getObjectsInCamera();

    for(auto no_data_obj : no_data_objects)
    {
      if(objects_in_camera.find(no_data_obj->bulletId()) != objects_in_camera.end())
        removeEntityPose(no_data_obj);
    }
  }

  UpdateAabbs();
}

std::vector<PointOfInterest> ObjectsPerceptionManager::getPoisInFov(Object* object)
{
  if(myself_agent_ == nullptr)
  {
    ShellDisplay::error("[ObjectsPerceptionManager] has no agent defined");
    return object->getPointsOfInterest();
  }
  if(myself_agent_->getHead() == nullptr)
  {
    ShellDisplay::error("[ObjectsPerceptionManager] defined agent has no head");
    return object->getPointsOfInterest();
  }
    

  std::vector<PointOfInterest> pois_in_fov;

  for(const auto& poi : object->getPointsOfInterest())
  {
    bool poi_is_valid = true;
    for(const auto& point : poi.getPoints())
    {
      auto poi_in_map = object->pose() * point;
      auto poi_in_head = poi_in_map.transformIn(myself_agent_->getHead()->pose());
      if (myself_agent_->getFieldOfView().hasIn(poi_in_head))
        continue;
      else
      {
        poi_is_valid = false;
        break;
      }
    }

    if(poi_is_valid)
      pois_in_fov.push_back(poi);
  }

  return pois_in_fov;
}

bool ObjectsPerceptionManager::shouldBeSeen(Object* object, const std::vector<PointOfInterest>& pois)
{
  if(myself_agent_->getHead() == nullptr)
    return false;

  for(const auto& poi : pois)
  {
    std::vector<std::array<double, 3>> from_poses(poi.getPoints().size(), myself_agent_->getHead()->pose().arrays().first);
    std::vector<std::array<double, 3>> to_poses;

    for(const auto& point : poi.getPoints()){
      Pose map_to_point = object->pose() * point;
      to_poses.push_back(map_to_point.arrays().first);
    }
    auto ray_cast_info = bullet_client_->rayTestBatch(from_poses, to_poses, poi.getPoints().size(), true);

    if(ray_cast_info.size() == 0)
      return true;
    /*else
    {
      for(auto& info : ray_cast_info)
      {
        bullet_client_->addUserDebugLine({info.m_hitPositionWorld[0],
                                          info.m_hitPositionWorld[1],
                                          info.m_hitPositionWorld[2]},
                                         myself_agent_->getHead()->pose().arrays().first,
                                         {1,0,0}, 1, 2.0);
        
      }
    }*/
  }

  return false;
}

std::unordered_set<int> ObjectsPerceptionManager::getObjectsInCamera()
{
  if(myself_agent_ == nullptr)
    return {};
  else if(myself_agent_->getHead() == nullptr)
    return {};
  else if(myself_agent_->getHead()->isLocated() == false)
    return {};

  auto proj_matrix = bullet_client_->computeProjectionMatrix(myself_agent_->getFieldOfView().getHeight(),
                                                              myself_agent_->getFieldOfView().getRatio(),
                                                              myself_agent_->getFieldOfView().getClipNear(),
                                                              myself_agent_->getFieldOfView().getClipFar());
  Pose target_pose = myself_agent_->getHead()->pose() * Pose({1,0,0}, {0,0,0,1});
  auto head_pose_trans = myself_agent_->getHead()->pose().arrays().first;
  auto target_pose_trans = target_pose.arrays().first;
  auto view_matrix = bullet_client_->computeViewMatrix({(float)head_pose_trans[0], (float)head_pose_trans[1], (float)head_pose_trans[2]},
                                                        {(float)target_pose_trans[0], (float)target_pose_trans[1], (float)target_pose_trans[2]},
                                                        {0.,0.,1.});
  auto images = bullet_client_->getCameraImage(175*myself_agent_->getFieldOfView().getRatio(), 175, view_matrix, proj_matrix, owds::BULLET_HARDWARE_OPENGL);

  return bullet_client_->getSegmentationIds(images);
}

} // namespace owds