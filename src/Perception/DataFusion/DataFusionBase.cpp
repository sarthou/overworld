#include "overworld/Perception/DataFusion/DataFusionBase.h"

#include "overworld/BasicTypes/Hand.h"

namespace owds {

  void DataFusionBase::fuseData(std::unordered_map<std::string, Percept<Object>*>& fusioned_percepts,
                                std::map<std::string, std::vector<Percept<Object>>>& aggregated_)
  {
    for(auto& pair : aggregated_)
    {
      if(pair.second.size() == 0)
        continue;

      auto fused_percept_it = fusioned_percepts.find(pair.first);
      if(fused_percept_it == fusioned_percepts.end())
      {
        fused_percept_it = fusioned_percepts.emplace(pair.first, new Percept<Object>(pair.second.front())).first;
        fused_percept_it->second->removeFromHand();
      }

      Percept<Object>* percept = fused_percept_it->second;
      std::string percept_id = pair.first;

      Hand* hand = nullptr;
      Pose pose_in_hand;
      Pose pose_in_map;
      int nb_frame_unseen = 1000;

      // We try to find if the percept should be in hand
      for(auto& obj : pair.second)
      {
        if(obj.isInHand())
        {
          hand = obj.getHandIn();
          if(obj.poseRaw().similarTo(Pose()) == false)
          {
            // The percept is in a hand with a stated transform
            pose_in_hand = obj.poseRaw();
            break;
          }
        }
        else if(obj.isLocated() && nb_frame_unseen >= obj.getNbFrameUnseen())
        {
          // We take the pose of the most recently perceived percept
          pose_in_map = obj.pose();
          nb_frame_unseen = obj.getNbFrameUnseen();
        }
      }

      if(hand != nullptr)
      {
        for(auto& obj : pair.second)
          percept->merge(&obj, false); // to update the shape but not the pose
        percept->setSeen();

        // If the precept was already in hand we do not have to update the transform
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
          else
            percept->updatePose(Pose());
        }
        else
          percept->updatePose(percept->poseRaw());
      }
      else
      {
        if(percept->isInHand())
        {
          Hand* hand = percept->getHandIn();
          auto pose_tmp = percept->pose();
          hand->removePerceptFromHand(percept->id());
          percept->updatePose(pose_tmp);
          nb_frame_unseen = 0; // releasing an object assume a perception
        }

        percept->setNbFrameUnseen(nb_frame_unseen);
        for(auto& obj : pair.second)
          percept->merge(&obj);
      }
    }
  }

} // namespace owds