#include "overworld/Perception/Managers/ObjectsPerceptionManager.h"

#include "overworld/BasicTypes/Hand.h"

#define TO_HALF_RAD M_PI / 180. / 2.

#define MAX_UNSEEN 2
#define MAX_SIMULATED 10
#define IN_HAND_DISTANCE 0.30 // meters

namespace owds {

std::map<std::string, Object*> ObjectsPerceptionManager::getEntities() const
{
  if(merged_ids_.size() == 0)
    return entities_;
  else
  {
    std::map<std::string, Object*> res;
    for(auto& entity : entities_)
    {
      if(merged_ids_.find(entity.first) == merged_ids_.end())
        res.insert(entity);
    }

    return res;
  }
}

void ObjectsPerceptionManager::updateSimulatedPoses()
{
  for(auto& simulated_object : simulated_objects_)
  {
    auto pose = bullet_client_->getBasePositionAndOrientation(entities_[simulated_object.first]->bulletId());
    entities_[simulated_object.first]->updatePose(pose.first, pose.second);
  }
}

void ObjectsPerceptionManager::initLerp()
{
  goal_poses_.clear();
  for(auto& object : entities_)
  {
    if(object.second->isStatic() == false)
    {
      if(object.second->isLocated())
      {
        if(simulated_objects_.find(object.second->id()) == simulated_objects_.end())
        {
          if(object.second->pose() != object.second->pose(1))
          {
              goal_poses_.insert({object.second, object.second->pose()});
              undoInBullet(object.second); // set to the previous pose
          }
        }
      }
    }
  }
}

void ObjectsPerceptionManager::stepLerp(double alpha)
{
  for(auto& goal_it : goal_poses_)
  {
    Pose new_pose = goal_it.first->pose(1).lerpTo(goal_it.second, alpha);
    goal_it.first->replacePose(new_pose);
    updateToBullet(goal_it.first);
  }
}

std::map<std::string, Object*>::iterator ObjectsPerceptionManager::createFromPercept(const Object& percept)
{
  auto new_object = new Object(percept);
  new_object->setInHand(nullptr);
  auto it = entities_.insert(std::pair<std::string, Object*>(percept.id(), new_object)).first;
  addToBullet(it->second);

  getObjectBoundingBox(it->second);
  if(it->second->getMass() == 0)
    it->second->setDefaultMass();

  it->second->setTypes(onto_->individuals.getUp(it->first));

  return it;
}

void ObjectsPerceptionManager::getPercepts(std::map<std::string, Object>& percepts)
{
  for(auto& percept : percepts)
  {
      if(percept.second.isTrueId() == false)
      {
        if(merged_ids_.find(percept.first) == merged_ids_.end())
          false_ids_to_be_merged_.insert(percept.first);
      }

      auto it = entities_.find(percept.second.id());
      if(it == entities_.end())
      {
          if(percept.second.isLocated() == false)
            continue;

          it = createFromPercept(percept.second);
      }

      if(merged_ids_.find(percept.first) != merged_ids_.end())
        continue;

      // reset lost_objects_nb_frames_
      if(percept.second.hasBeenSeen())
        lost_objects_nb_frames_.erase(percept.first);

      if(percept.second.isInHand() && (it->second->isInHand() == false))
      {
        // this is a big shit, I know that
        auto hand = percept.second.getHandIn();
        percept.second.removeFromHand();
        hand->putInHand(it->second);
        percept.second.setInHand(hand);
        stopSimulation(it->second);
      }
      
      if(it->second->isInHand())
      {
        if(it->second->getHandIn()->isInHand(it->first) == false)
        {
          it->second->removeFromHand();
        }
        else if (percept.second.hasBeenSeen() && it->second->getHandIn()->pose().distanceTo(percept.second.pose()) >= IN_HAND_DISTANCE)
        {
          it->second->removeFromHand();
          updateEntityPose(it->second, percept.second.pose(), percept.second.lastStamp());
        }
        else if(percept.second.isLocated() == false)
        {
          it->second->removeFromHand();
        }
        else
        {
          updateEntityPose(it->second, it->second->getHandIn()->pose(), ros::Time::now());
        }
      }
      // TODO : The following assume that an entity is perceive by only one module
      // A true merging algorithm has to be applied
      else if (percept.second.hasBeenSeen())
      {
        stopSimulation(it->second);
        updateEntityPose(it->second, percept.second.pose(), percept.second.lastStamp());
      }
      else
      {
        if(it->second->isLocated())
          it->second->updatePose(it->second->pose(), ros::Time::now());
      }
  }
}

bool ObjectsPerceptionManager::souldBeReasonedOn(Object* object)
{
  if(object->isStatic() == true)
    return false;
  else if (object->isLocated() == false)
    return false;
  else if (object->isInHand() == true)
    return false;
  else if (object->getNbFrameUnseen() < MAX_UNSEEN)
    return false;
  else
    return true;
}

void ObjectsPerceptionManager::reasoningOnUpdate()
{
  bullet_client_->performCollisionDetection();
  UpdateAabbs();

  if(false_ids_to_be_merged_.size())
    mergeFalseIdData();

  for(auto& merged : merged_ids_)
  {
    entities_.at(merged.second)->merge(entities_.at(merged.first));
    removeEntityPose(entities_.at(merged.first));
  }

  std::vector<Object*> no_data_objects;
  std::vector<Object*> objects_to_remove;
  std::vector<Object*> objects_to_simulate;
  for(auto& object : entities_)
  {
    if(souldBeReasonedOn(object.second) == false)
    {
      if(object.second->isStatic() == false)
        lost_objects_nb_frames_.erase(object.first);
      continue;
    }

    if(object.second->getPointsOfInterest().size() != 0)
    {
      auto pois_in_fov = getPoisInFov(object.second);
      if(pois_in_fov.size() != 0)
      {
        if(shouldBeSeen(object.second, pois_in_fov))
        {
          auto it_unseen = lost_objects_nb_frames_.find(object.first);
          if(it_unseen == lost_objects_nb_frames_.end())
            it_unseen = lost_objects_nb_frames_.insert({object.second->id(), 0}).first;

          it_unseen->second++;
          if(it_unseen->second > MAX_UNSEEN)
            objects_to_remove.push_back(object.second);
        }
        else
          objects_to_simulate.push_back(object.second);
      }
    }
    else // Object has no poi
      no_data_objects.push_back(object.second);
  }

  if(no_data_objects.size())
  {
    if(isObjectsInFovAabb(no_data_objects))
    {
      auto objects_in_camera = getObjectsInCamera();

      for(auto no_data_obj : no_data_objects)
      {
        // equivalent to shouldBeSeen
        if(objects_in_camera.find(no_data_obj->bulletId()) != objects_in_camera.end())
        {
          auto it_unseen = lost_objects_nb_frames_.find(no_data_obj->id());
          if(it_unseen == lost_objects_nb_frames_.end())
            it_unseen = lost_objects_nb_frames_.insert({no_data_obj->id(), 0}).first;

          it_unseen->second++;
          if(it_unseen->second > MAX_UNSEEN)
          objects_to_remove.push_back(no_data_obj);
        }
        else
          objects_to_simulate.push_back(no_data_obj);
      }
    }
  }

  // From there, objects to be removed are in the agent Fov but we don't have
  // any information about them neither any explanation
  objects_to_remove = simulatePhysics(objects_to_remove, objects_to_simulate);

  for(auto obj : objects_to_remove)
    removeEntityPose(obj);
}

std::vector<Object*> ObjectsPerceptionManager::simulatePhysics(const std::vector<Object*>& lost_objects, const std::vector<Object*>& to_simulate_objetcs)
{
  if(simulate_)
  {
    std::unordered_set<std::string> lost_ids;
    lost_ids.reserve(lost_objects.size());
    for(auto& object : lost_objects)
    {
      lost_ids.insert(object->id());
      auto it = simulated_objects_.find(object->id());
      if(it == simulated_objects_.end())
        startSimulation(object);
    }

    for(auto& object : to_simulate_objetcs)
    {
      auto it = simulated_objects_.find(object->id());
      if(it == simulated_objects_.end())
        startSimulation(object);
    }

    std::vector<Object*> objects_to_remove;
    for(auto& simulated_object : simulated_objects_)
    {
      if(lost_ids.find(simulated_object.first) == lost_ids.end())
        simulated_object.second = 0;
      else
      {
        simulated_object.second++;
        if(simulated_object.second > MAX_SIMULATED)
        {
          auto entity = entities_[simulated_object.first];
          stopSimulation(entity, false);
          objects_to_remove.push_back(entity);
        }
      }
    }

    // We remove them in a second time as we loop on them previously
    for(auto entity : objects_to_remove)
      simulated_objects_.erase(entity->id());

    return objects_to_remove;
  }
  else
  {
    std::vector<Object*> objects_to_remove = lost_objects;
    objects_to_remove.insert(objects_to_remove.end(), to_simulate_objetcs.begin(), to_simulate_objetcs.end());
    return objects_to_remove;
  }
}

void ObjectsPerceptionManager::startSimulation(Object* object)
{
  std::cout << "--start simulation for " << object->id() << " with a mass of " << object->getMass() << std::endl;
  bullet_client_->setMass(object->bulletId(), -1, object->getMass());
  bullet_client_->resetBaseVelocity(object->bulletId(), {0,0,0}, {0,0,0});

  simulated_objects_.insert({object->id(), 0});

  /*bullet_client_->performCollisionDetection();
  auto contact_points = bullet_client_->getContactPoints(object->bulletId());
  std::cout << "==> " << contact_points.m_numContactPoints << " CPs" << std::endl;
  for(size_t i = 0; i < contact_points.m_numContactPoints; i++)
  {
    auto point = contact_points.m_contactPointData[i];
    auto contact_entity = getEntityFromBulletId(point.m_bodyUniqueIdB);
    if(contact_entity != nullptr)
      std::cout << "====> contact with " << contact_entity->id() << " on dist " << point.m_contactDistance << std::endl;
  }*/
}

void ObjectsPerceptionManager::stopSimulation(Object* object, bool erase)
{
  auto it = simulated_objects_.find(object->id());
  if(it != simulated_objects_.end())
  {
    std::cout << "--stop simulation for " << object->id() << std::endl;
    bullet_client_->setMass(object->bulletId(), -1, 0);
    if(erase)
      simulated_objects_.erase(object->id());
  }
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

bool ObjectsPerceptionManager::isObjectsInFovAabb(std::vector<Object*> objects)
{
  for(auto object : objects)
  {
    std::array<Pose, 8> points = { Pose({object->getAabb().min[0], object->getAabb().min[1], object->getAabb().min[2]}, {0,0,0,1}),
                                   Pose({object->getAabb().min[0], object->getAabb().min[1], object->getAabb().max[2]}, {0,0,0,1}),
                                   Pose({object->getAabb().min[0], object->getAabb().max[1], object->getAabb().min[2]}, {0,0,0,1}),
                                   Pose({object->getAabb().min[0], object->getAabb().max[1], object->getAabb().max[2]}, {0,0,0,1}),
                                   Pose({object->getAabb().max[0], object->getAabb().min[1], object->getAabb().min[2]}, {0,0,0,1}),
                                   Pose({object->getAabb().max[0], object->getAabb().min[1], object->getAabb().max[2]}, {0,0,0,1}),
                                   Pose({object->getAabb().max[0], object->getAabb().max[1], object->getAabb().min[2]}, {0,0,0,1}),
                                   Pose({object->getAabb().max[0], object->getAabb().max[1], object->getAabb().max[2]}, {0,0,0,1}) };

    for(const auto& point : points)
    {
      auto point_in_head = point.transformIn(myself_agent_->getHead()->pose());
      if (myself_agent_->getFieldOfView().hasIn(point_in_head))
        return true;
    }
  }

  return false;
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
                                                              myself_agent_->getFieldOfView().getRatioOpenGl(),
                                                              myself_agent_->getFieldOfView().getClipNear(),
                                                              myself_agent_->getFieldOfView().getClipFar());
  Pose target_pose = myself_agent_->getHead()->pose() * Pose({0,0,1}, {0,0,0,1});
  auto head_pose_trans = myself_agent_->getHead()->pose().arrays().first;
  auto target_pose_trans = target_pose.arrays().first;
  auto view_matrix = bullet_client_->computeViewMatrix({(float)head_pose_trans[0], (float)head_pose_trans[1], (float)head_pose_trans[2]},
                                                        {(float)target_pose_trans[0], (float)target_pose_trans[1], (float)target_pose_trans[2]},
                                                        {0.,0.,1.});
  auto images = bullet_client_->getCameraImage(100*myself_agent_->getFieldOfView().getRatioOpenGl(), 100, view_matrix, proj_matrix, owds::BULLET_HARDWARE_OPENGL);
  return bullet_client_->getSegmentationIds(images);
}

void ObjectsPerceptionManager::mergeFalseIdData()
{
  std::unordered_set<std::string> merged;

  for(auto& id : false_ids_to_be_merged_)
  {
    auto obj = entities_.find(id);
    if(obj->second->isLocated() == false)
      continue;

    double obj_volume = obj->second->getAabbVolume();
    double min_error = 10000;
    Object* to_be_merged = nullptr;

    for(auto entity : entities_)
    {
      if(entity.second->isStatic())
        continue;
      else if(entity.second->isLocated() == false)
        continue;
      else if(merged_ids_.find(entity.first) != merged_ids_.end())
        continue;

      if(entity.first != obj->first)
      {
        if(entity.second->pose().distanceTo(obj->second->pose()) <= 0.1) // TODO tune
        {
          double error = std::abs(obj_volume - entity.second->getAabbVolume());
          if(error < min_error)
          {
            to_be_merged = entity.second;
            min_error = error;
          }
        }
      }
    }

    if(to_be_merged != nullptr)
    {
      merged.insert(id);
      to_be_merged->merge(obj->second);
      to_be_merged->addFalseId(id);
      merged_ids_.insert(std::make_pair(id, to_be_merged->id()));
    }
  }

  for(auto& id : merged)
    false_ids_to_be_merged_.erase(id);
}

void ObjectsPerceptionManager::getObjectBoundingBox(Object* object)
{
  if(object->isLocated() == false)
    return;

  auto tmp_pose = object->pose();
  updateEntityPose(object, {{0,0,0}, {0,0,0,1}}, ros::Time::now());

  bullet_client_->performCollisionDetection();
  auto bb = bullet_client_->getAABB(object->bulletId());

  updateEntityPose(object, tmp_pose, ros::Time::now());

  object->setBoundingBox({bb.max[0] - bb.min[0], bb.max[1] - bb.min[1], bb.max[2] - bb.min[2]});
  object->setOriginOffset({(bb.max[0] - bb.min[0]) / 2. + bb.min[0],
                           (bb.max[1] - bb.min[1]) / 2. + bb.min[1],
                           (bb.max[2] - bb.min[2]) / 2. + bb.min[2]});
  object->computeCorners();
}

} // namespace owds