#include "overworld/Perception/DataFusion/DataFusionBase.h"

#include "overworld/BasicTypes/Hand.h"

namespace owds {

  void DataFusionBase::fuseData(std::unordered_map<std::string, Percept<Object>*>& fusioned_percepts,
                                std::map<std::string, std::map<std::string, Percept<Object>>>& aggregated_)
  {
    for(auto& it : aggregated_)
    {
      if(it.second.size() == 0)
        continue;

      auto fused_percept_it = fusioned_percepts.find(it.first);
      if(fused_percept_it == fusioned_percepts.end())
      {
        fused_percept_it = fusioned_percepts.emplace(it.first, new Percept<Object>(it.second.begin()->second)).first;
        fused_percept_it->second->removeFromHand();
      }

      Percept<Object>* percept = fused_percept_it->second;
      percept->setSensorId(it.second.begin()->second.getSensorId()); // we initialize the sensor_id of the percept created
      percept->setModuleName(it.second.begin()->second.getModuleName());
      std::string percept_id = it.first;

      Hand* hand = nullptr;
      Pose pose_in_hand;
      Pose pose_in_map;
      bool located_in_map = false;
      size_t nb_frame_unseen = 1000;
      float best_confidence = 0.;
      Percept<Object>* percept_to_merge = nullptr;

      // We try to find if the percept should be in hand
      for(auto& inner_it : it.second)
      {
        if(inner_it.second.isInHand())
        {
          hand = inner_it.second.getHandIn();
          if(inner_it.second.poseRaw().similarTo(Pose()) == false)
          {
            // The percept is in a hand with a stated transform
            pose_in_hand = inner_it.second.poseRaw();
            break;
          }
        }
        else if(inner_it.second.isLocated() && nb_frame_unseen >= inner_it.second.getNbFrameUnseen())
        {
          // We take the pose of the most recently perceived percept
          pose_in_map = inner_it.second.pose();
          located_in_map = true;

          if(nb_frame_unseen > inner_it.second.getNbFrameUnseen())
            best_confidence = 0.;
          nb_frame_unseen = inner_it.second.getNbFrameUnseen();

          if(inner_it.second.getConfidence() >= best_confidence)
          {
            best_confidence = inner_it.second.getConfidence();
            percept_to_merge = &inner_it.second;
          }
        }
      }

      if(hand != nullptr)
      {
        for(auto& inner_it : it.second)
          percept->merge(&inner_it.second, false); // to update the shape but not the pose
        percept->setSeen();
        // If the percept was already in hand we do not have to update the transform
        if(percept->isInHand() == false)
        {
          hand->putPerceptInHand(percept);
          if(pose_in_hand.similarTo(Pose()) == false)
            percept->updatePose(pose_in_hand);
          else if(pose_in_map.similarTo(Pose()) == false)
          {
            pose_in_hand = pose_in_map.transformIn(hand->pose());
            percept->updatePose(pose_in_hand);
          }
          // else
          // percept->updatePose(Pose());
        }
        // else
        // percept->updatePose(percept->poseRaw()); //update pose in hand
      }
      else
      {
        if(percept->isInHand())
        {
          Hand* hand = percept->getHandIn();
          auto pose_tmp = percept->pose();
          hand->removePerceptFromHand(percept->id());
          for(auto& false_id : percept->getFalseIds())
            hand->removePerceptFromHand(false_id);

          if(located_in_map)
            percept->updatePose(pose_in_map);
          else
            percept->updatePose(pose_tmp);

          // percept->updatePose(pose_tmp);
          // nb_frame_unseen = 0;
        }

        percept->setNbFrameUnseen(nb_frame_unseen);
        for(auto& inner_it : it.second)
          percept->merge(&inner_it.second, false);

        if(percept_to_merge != nullptr)
          percept->merge(percept_to_merge, true);
      }
    }
  }

} // namespace owds
