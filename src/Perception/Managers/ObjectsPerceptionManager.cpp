#include "overworld/Perception/Managers/ObjectsPerceptionManager.h"

#include <array>
#include <string>
#include <vector>
#include <map>
#include <cstddef>

#include "overworld/BasicTypes/Hand.h"

#define TO_HALF_RAD M_PI / 180. / 2.

#define MAX_UNSEEN 2
#define MAX_SIMULATED 10
#define IN_HAND_DISTANCE 0.30 // meters

namespace owds {

  void ObjectsPerceptionManager::updateSimulatedPoses()
  {
    for(auto& simulated_object : simulated_objects_)
    {
      auto pose = world_client_->getBasePositionAndOrientation(entities_[simulated_object.first]->worldId());
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
        if(object.second->isLocated() && (object.second->isInHand() == false))
        {
          if(simulated_objects_.find(object.second->id()) == simulated_objects_.end())
          {
            if(object.second->pose() != object.second->pose(1))
            {
              goal_poses_.insert({object.second, object.second->poseRaw()});
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

  std::map<std::string, Object*>::iterator ObjectsPerceptionManager::createFromFusedPercept(Percept<Object>* percept)
  {
    auto new_object = new Object(*percept);
    auto it = entities_.emplace(percept->id(), new_object).first;
    addToWorld(it->second);

    new_object->removeFromHand(); // We remove in case an entity is created from a percept already in hand

    getObjectBoundingBox(it->second);
    if(it->second->getMass() == 0)
      it->second->setDefaultMass();
    world_client_->setMass(it->second->worldId(), -1, it->second->getMass());

    it->second->setTypes(onto_->individuals.getUp(it->first));

    return it;
  }

  void ObjectsPerceptionManager::getPercepts(const std::string& module_name, std::map<std::string, Percept<Object>>& percepts)
  {
    for(auto& percept : percepts)
    {
      percept.second.setModuleName(module_name);

      if(percept.second.getSensorId().empty() == false) // we avoid problems with static object
      {
        auto* sensor = myself_agent_->getSensor(percept.second.getSensorId());
        if(sensor != nullptr)
          sensor->setPerceptseen(percept.first);
      }
      else if(myself_agent_->getType() == AgentType_e::HUMAN)
      {
        auto human_sensors = myself_agent_->getSensors();
        if(human_sensors.empty() == false)
        {
          auto sensor = human_sensors.begin()->second; // we assume a single sensor by human
          percept.second.setSensorId(sensor->id());
          sensor->setPerceptseen(percept.first);
        }
      }

      std::string entity_id = percept.first;
      if(percept.second.isTrueId() == false)
      {
        auto merged_id = merged_ids_.find(percept.first);
        if(merged_id == merged_ids_.end())
          false_ids_to_be_merged_.insert(percept.first);
        else
          entity_id = merged_id->second;
      }

      fusionAggregated(entity_id, module_name, percept.second);

      if(percept.second.getSensorId().empty() == false)
        fusionRegister(percept.first, percept.second.getSensorId(), percept.second.getModuleName());
    }
  }

  bool ObjectsPerceptionManager::shouldBeReasonedOn(Object* object)
  {
    if(object->isStatic() == true)
      return false;
    else if(object->isLocated() == false)
      return false;
    else if(object->isInHand() == true)
      return false;
    else if(object->getNbFrameUnseen() < MAX_UNSEEN)
      return false;
    else
      return true;
  }

  void ObjectsPerceptionManager::reasoningOnUpdate()
  {
    fusioner_.fuseData(fusioned_percepts_, aggregated_);

    if(false_ids_to_be_merged_.size())
      mergeFalseIdData();

    fromfusedToEntities();
    geometricReasoning();
  }

  bool ObjectsPerceptionManager::handReasoning(Percept<Object>* percept, Object* entity)
  {
    bool hand_reasoned = false;

    if(percept->isInHand() && (entity->isInHand() == false))
    {
      auto* hand = percept->getHandIn();
      hand->putInHand(entity);
      updateEntityPose(entity, percept->poseRaw(), percept->lastStamp());
      stopSimulation(entity);

      hand_reasoned = true;
    }
    else if(entity->isInHand())
    {
      auto* hand = entity->getHandIn();
      if(percept->hasBeenSeen() && hand->pose().distanceTo(percept->pose()) >= IN_HAND_DISTANCE)
      {
        hand->removeFromHand(entity->id());
        updateEntityPose(entity, percept->pose(), percept->lastStamp());
      }
      else if((percept->isLocated() == false) || (percept->isInHand() == false))
      {
        Pose obj_in_map = entity->pose();
        hand->removeFromHand(entity->id());
        updateEntityPose(entity, obj_in_map, ros::Time::now());
      }
      else
        updateEntityPose(entity, entity->poseRaw(), ros::Time::now());

      hand_reasoned = true;
    }

    return hand_reasoned;
  }

  void ObjectsPerceptionManager::fromfusedToEntities()
  {
    for(const auto& percept : fusioned_percepts_)
    {
      auto it = entities_.find(percept.second->id());
      if(it == entities_.end())
      {
        if(percept.second->isLocated() == false)
          continue;

        it = createFromFusedPercept(percept.second);
      }

      if(percept.second->hasBeenSeen())
        lost_objects_nb_frames_.erase(percept.first);

      if(handReasoning(percept.second, it->second) == false)
      {
        if(percept.second->hasBeenSeen())
        {
          stopSimulation(it->second);
          if(percept.second->isLocated())
            updateEntityPose(it->second, percept.second->pose(), percept.second->lastStamp());
        }
        else if(it->second->isLocated())
          it->second->updatePose(it->second->pose(), ros::Time::now());
      }
    }
  }

  void ObjectsPerceptionManager::geometricReasoning()
  {
    updateAabbs();

    std::map<std::string, std::set<Sensor*>> object_sensors_set; // map of the object and the sensors associated to module without poi
    std::map<std::string, Object*> no_data_objects;
    std::map<std::string, Object*> objects_to_remove;
    std::map<std::string, Object*> objects_to_simulate_oclusion;

    for(auto& object : entities_)
    {
      bool object_is_consider_with_no_poi = false;
      bool can_pass_to_the_next_object = false;
      if(shouldBeReasonedOn(object.second) == false)
      {
        if(object.second->isStatic() == false)
          lost_objects_nb_frames_.erase(object.first);
        continue;
      }

      auto it = entities_percetion_register_.find(object.first);
      if(it == entities_percetion_register_.end())
        continue;
      for(auto& pair_sensor_set_modules : it->second)
      {
        object_is_consider_with_no_poi = false;

        if(can_pass_to_the_next_object)
          break;

        for(auto& module_name : pair_sensor_set_modules.second)
        {
          if(object.second->getPointsOfInterest(module_name).size() != 0)
          {
            auto pois_in_fov = getPoisInFov(object.second, myself_agent_->getSensor(pair_sensor_set_modules.first), module_name);
            if(pois_in_fov.empty() == false)
            {
              if(shouldBeSeen(object.second, myself_agent_->getSensor(pair_sensor_set_modules.first), pois_in_fov))
              {
                auto it_unseen = lost_objects_nb_frames_.find(object.first);
                if(it_unseen == lost_objects_nb_frames_.end())
                  it_unseen = lost_objects_nb_frames_.insert({object.second->id(), 0}).first;

                it_unseen->second++;
                if(it_unseen->second > MAX_UNSEEN)
                {
                  objects_to_remove.emplace(object.first, object.second);
                  objects_to_simulate_oclusion.erase(object.first);
                  no_data_objects.erase(object.first);
                  can_pass_to_the_next_object = true;
                  break;
                }
              }
              else
              {
                objects_to_simulate_oclusion.emplace(object.first, object.second);
                no_data_objects.erase(object.first);
              }
            } // object with pois but none in the fov
            else // object with no pois in the fov should be put in no_data and erased if pois are in the fov of another module
            {
              // object_is_consider_with_no_poi = true;
            }
          }
          else // object has no pois
            object_is_consider_with_no_poi = true;
        }

        if(object_is_consider_with_no_poi)
        {
          no_data_objects.emplace(object.first, object.second);
          auto it = object_sensors_set.find(object.first);
          if(it == object_sensors_set.end())
            object_sensors_set.emplace(object.first, std::set<Sensor*>{myself_agent_->getSensor(pair_sensor_set_modules.first)});
          else
            it->second.emplace(myself_agent_->getSensor(pair_sensor_set_modules.first));
        }
      }
    }

    if(no_data_objects.size())
    {
      std::unordered_set<Sensor*> used_sensors;
      for(auto no_data_obj : no_data_objects)
      {
        auto sensor_set = object_sensors_set.find(no_data_obj.first);
        for(auto sensor : sensor_set->second)
          used_sensors.insert(sensor);
      }

      std::vector<int> sensor_world_ids;
      sensor_world_ids.reserve(used_sensors.size());
      for(auto sensor : used_sensors)
      {
        addToWorld(sensor);
        sensor_world_ids.push_back(sensor->getWorldSegmentationId());
        auto sensor_pose = sensor->pose().arrays();
        world_client_->setCameraPositionAndOrientation(sensor->getWorldSegmentationId(),
                                                       sensor_pose.first, sensor_pose.second);
      }
      world_client_->requestCameraRender(sensor_world_ids);
      std::unordered_map<Sensor*, std::unordered_set<uint32_t>> segmentations;
      for(auto sensor : used_sensors)
      {
        int sensor_id = sensor->getWorldSegmentationId();
        segmentations.emplace(sensor, world_client_->getCameraSementation(sensor_id));
      }

      for(auto no_data_obj : no_data_objects)
      {
        auto sensor_set = object_sensors_set.find(no_data_obj.first);
        for(auto sensor : sensor_set->second)
        {
          if((sensor != nullptr) && (sensor->isLocated()) &&
             isObjectInFovAabb(no_data_obj.second, sensor) && 
             (segmentations[sensor].find(no_data_obj.second->worldId()) != segmentations[sensor].end()))
          {
            auto it_unseen = lost_objects_nb_frames_.find(no_data_obj.first);
            if(it_unseen == lost_objects_nb_frames_.end())
              it_unseen = lost_objects_nb_frames_.insert({no_data_obj.first, 0}).first;

            it_unseen->second++;
            if(it_unseen->second > MAX_UNSEEN)
            {
              objects_to_remove.emplace(no_data_obj.first, no_data_obj.second);
              objects_to_simulate_oclusion.erase(no_data_obj.first);
              break;
            }
          }
          else // there is the case where one told to simulate and the next one says to remove, we wait the end of the loop to be sure
            objects_to_simulate_oclusion.emplace(no_data_obj.first, no_data_obj.second);
        }
      }
    }

    // From there, objects to be removed are in the agent Fov but we don't have
    // any information about them neither any explanation
    objects_to_remove = simulatePhysics(objects_to_remove, objects_to_simulate_oclusion);

    for(auto obj : objects_to_remove)
      removeEntityPose(obj.second);
  }

  std::map<std::string, Object*> ObjectsPerceptionManager::simulatePhysics(const std::map<std::string, Object*>& lost_objects,
                                                                           const std::map<std::string, Object*>& objects_to_simulate_oclusion)
  {
    if(simulate_)
    {
      std::unordered_set<std::string> lost_ids;
      lost_ids.reserve(lost_objects.size());
      for(auto& object : lost_objects)
      {
        lost_ids.insert(object.first);
        auto it = simulated_objects_.find(object.first);
        if(it == simulated_objects_.end())
          startSimulation(object.second);
      }

      for(auto& object : objects_to_simulate_oclusion)
      {
        auto it = simulated_objects_.find(object.first);
        if(it == simulated_objects_.end())
          startSimulation(object.second);
      }

      std::map<std::string, Object*> objects_to_remove;
      for(auto& simulated_object : simulated_objects_)
      {
        if(lost_ids.find(simulated_object.first) == lost_ids.end())
          simulated_object.second = 0;
        else
        {
          simulated_object.second++;
          if(simulated_object.second > MAX_SIMULATED)
          {
            auto entity = entities_.find(simulated_object.first);
            stopSimulation(entity->second, false);
            objects_to_remove.emplace(entity->first, entity->second);
          }
        }
      }

      // We remove them in a second time as we loop on them previously
      for(auto entity : objects_to_remove)
        simulated_objects_.erase(entity.first);

      return objects_to_remove;
    }
    else
      return lost_objects;
  }

  void ObjectsPerceptionManager::startSimulation(Object* object)
  {
    std::cout << "--start simulation for " << object->id() << " with a mass of " << object->getMass() << std::endl;
    //world_client_->setMass(object->worldId(), -1, object->getMass());
    world_client_->setBaseVelocity(object->worldId(), {0, 0, 0}, {0, 0, 0});
    world_client_->setSimulation(object->worldId(), true);

    simulated_objects_.insert({object->id(), 0});
  }

  void ObjectsPerceptionManager::stopSimulation(Object* object, bool erase)
  {
    auto it = simulated_objects_.find(object->id());
    if(it != simulated_objects_.end())
    {
      std::cout << "--stop simulation for " << object->id() << std::endl;
      //world_client_->setMass(object->worldId(), -1, 0);
      world_client_->setSimulation(object->worldId(), false);
      if(erase)
        simulated_objects_.erase(object->id());
    }
  }

  std::vector<PointOfInterest> ObjectsPerceptionManager::getPoisInFov(Object* object, Sensor* sensor, std::string module_name)
  {
    if(sensor == nullptr)
    {
      ShellDisplay::error("[ObjectsPerceptionManager] has no sensor defined");
      return object->getPointsOfInterest(module_name);
    }
    if(sensor->id().empty())
    {
      ShellDisplay::error("[ObjectsPerceptionManager] defined sensor is empty");
      return object->getPointsOfInterest(module_name);
    }

    std::vector<PointOfInterest> pois_in_fov;

    for(const auto& poi : object->getPointsOfInterest(module_name))
    {
      bool poi_is_valid = true;
      for(const auto& point : poi.getPoints())
      {
        auto poi_in_map = object->pose() * point;
        auto poi_in_sensor = poi_in_map.transformIn(sensor->pose());
        if(sensor->getFieldOfView().hasIn(poi_in_sensor))
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

  bool ObjectsPerceptionManager::isObjectInFovAabb(Object* object, Sensor* sensor)
  {
    size_t nb_in_fov = 0;
    std::array<Pose, 8> points = {Pose({object->getAabb().min[0], object->getAabb().min[1], object->getAabb().min[2]}, {0, 0, 0, 1}),
                                  Pose({object->getAabb().min[0], object->getAabb().min[1], object->getAabb().max[2]}, {0, 0, 0, 1}),
                                  Pose({object->getAabb().min[0], object->getAabb().max[1], object->getAabb().min[2]}, {0, 0, 0, 1}),
                                  Pose({object->getAabb().min[0], object->getAabb().max[1], object->getAabb().max[2]}, {0, 0, 0, 1}),
                                  Pose({object->getAabb().max[0], object->getAabb().min[1], object->getAabb().min[2]}, {0, 0, 0, 1}),
                                  Pose({object->getAabb().max[0], object->getAabb().min[1], object->getAabb().max[2]}, {0, 0, 0, 1}),
                                  Pose({object->getAabb().max[0], object->getAabb().max[1], object->getAabb().min[2]}, {0, 0, 0, 1}),
                                  Pose({object->getAabb().max[0], object->getAabb().max[1], object->getAabb().max[2]}, {0, 0, 0, 1})};

    for(const auto& point : points)
    {
      auto point_in_sensor = point.transformIn(sensor->pose());
      if(sensor->getFieldOfView().hasIn(point_in_sensor, 2))
      {
        nb_in_fov++;
        if(nb_in_fov >= 2)
          return true;
      }
    }
    return false;
  }

  bool ObjectsPerceptionManager::shouldBeSeen(Object* object, Sensor* sensor, const std::vector<PointOfInterest>& pois)
  {
    if(sensor->id().empty() || sensor == nullptr)
      return false;

    for(const auto& poi : pois) // TODO not all poi are considered
    {
      std::vector<std::array<double, 3>> from_poses(poi.getPoints().size(), sensor->pose().arrays().first);
      std::vector<std::array<double, 3>> to_poses;
      //std::vector<std::array<double, 3>> debug_vertices;
      //std::vector<unsigned int> debug_index_;
      double max_dist = 0;

      for(const auto& point : poi.getPoints())
      {
        Pose map_to_point = object->pose() * point;
        double poi_dist = map_to_point.distanceTo(sensor->pose()) + 0.01;
        if(poi_dist > max_dist)
          max_dist = poi_dist;

        to_poses.push_back(map_to_point.arrays().first);
        //debug_vertices.push_back(map_to_point.arrays().first);
        //debug_vertices.push_back(sensor->pose().arrays().first);
        //debug_index_.push_back(debug_index_.size());
        //debug_index_.push_back(debug_index_.size());
      }
      //world_client_->addDebugLine(debug_vertices, debug_index_, {0., 1., 0.}, 0.5);
      if(max_dist > sensor->getFieldOfView().getClipFar())
        max_dist = sensor->getFieldOfView().getClipFar();
      auto ray_cast_info = world_client_->raycasts(from_poses, to_poses, max_dist);

      if(ray_cast_info.empty())
        return true;
      else
      {
        for(auto& info : ray_cast_info)
        {
          if(info.actor_id != object->worldId())
          {
            //world_client_->addDebugLine(sensor->pose().arrays().first, info.position, {1., 0., 0.}, 0.5);
            return false;
          }
        }
        return true;
      }
    }

    return false;
  }

  void ObjectsPerceptionManager::mergeFalseIdData()
  {
    double direction_similarity_threshold = 0;
    double speed_difference_threshold = 5;
    int times_seen_difference_threshold = 20;

    std::unordered_set<std::string> merged;

    for(auto& false_id : false_ids_to_be_merged_)
    {
      auto false_obj = fusioned_percepts_.find(false_id);
      if(false_obj->second->isLocated() == false)
        continue;

      double obj_volume = false_obj->second->getBbVolume();
      double min_error = 10000;
      Percept<Object>* to_be_merged = nullptr;

      for(auto& percept : fusioned_percepts_)
      {
        if(percept.second->isStatic())
          continue;
        else if(percept.second->isLocated() == false)
          continue;
        else if(percept.second->getBbVolume() == 0) // not yet initialized
          continue;
        else if(merged_ids_.find(percept.first) != merged_ids_.end()) // we cannot merge two false ids together
          continue;

        if(percept.first != false_obj->first)
        {
          if(percept.second->pose().distanceTo(false_obj->second->pose()) <= 0.1) // TODO tune
          {
            double error = std::abs(obj_volume - percept.second->getBbVolume());
            if(error < min_error)
            {
              to_be_merged = percept.second;
              min_error = error;
            }
          }
        }
      }

      if(to_be_merged != nullptr)
      {
        fusionRegister(to_be_merged->id(), false_obj->second->getSensorId(), false_obj->second->getModuleName());
        merged.insert(false_id);
        to_be_merged->addFalseId(false_id);
        merged_ids_.insert(std::make_pair(false_id, to_be_merged->id()));
      }
    }

    // for all merged false entities, we unset its pose (put it below the world)
    // then we erase it from all maps
    for(auto& false_id : merged)
    {
      entities_percetion_register_.erase(false_id);
      false_ids_to_be_merged_.erase(false_id);
      aggregated_.erase(false_id);

      auto to_be_removed = fusioned_percepts_.at(false_id);

      delete to_be_removed;
      fusioned_percepts_.erase(false_id);

      if(entities_.find(false_id) != entities_.end())
      {
        removeEntityPose(entities_.at(false_id));
        delete entities_.at(false_id);
        entities_.erase(false_id);
      }
    }
  }

  void ObjectsPerceptionManager::getObjectBoundingBox(Object* object)
  {
    if(object->isLocated() == false)
      return;

    auto bb = world_client_->getLocalAABB(object->worldId());
    if(bb.isValid() == false)
      return;

    object->setBoundingBox({bb.max[0] - bb.min[0], bb.max[1] - bb.min[1], bb.max[2] - bb.min[2]});
    object->setOriginOffset({(bb.max[0] - bb.min[0]) / 2. + bb.min[0],
                             (bb.max[1] - bb.min[1]) / 2. + bb.min[1],
                             (bb.max[2] - bb.min[2]) / 2. + bb.min[2]});
    object->computeCorners();

    auto percept_it = fusioned_percepts_.find(object->id());
    if(percept_it != fusioned_percepts_.end())
    {
      percept_it->second->setBoundingBox({bb.max[0] - bb.min[0], bb.max[1] - bb.min[1], bb.max[2] - bb.min[2]});
      percept_it->second->setOriginOffset({(bb.max[0] - bb.min[0]) / 2. + bb.min[0],
                                           (bb.max[1] - bb.min[1]) / 2. + bb.min[1],
                                           (bb.max[2] - bb.min[2]) / 2. + bb.min[2]});
      percept_it->second->computeCorners();
    }
  }

} // namespace owds