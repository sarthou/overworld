#include "overworld/Perception/Managers/RobotsPerceptionManager.h"

#include "overworld/BasicTypes/Hand.h"
#include "overworld/Utils/ShellDisplay.h"

namespace owds {

  void RobotsPerceptionManager::getPercepts(const std::string& module_name, std::map<std::string, Percept<BodyPart>>& percepts)
  {
    for(auto& percept : percepts)
    {
      if(percept.second.getType() == owds::BODY_PART_SENSOR)
      {
        auto it = sensors_register_.find(percept.first);
        if(it == sensors_register_.end())
        {
          auto frame_it = frames_register_.find(percept.first);
          if(frame_it != frames_register_.end())
            it = sensors_register_.find(frame_it->second->id());
        }

        if(it == sensors_register_.end())
        {
          auto vect = onto_->individuals.getFrom("hasFrameId", percept.first);
          if(vect.empty())
            vect = onto_->individuals.find(percept.first);

          it = createSensor(vect.front(), percept.second.getAgentName());
          updateAgent(it->second, AgentType_e::ROBOT);
        }
        if(percept.second.isLocated())
          it->second->updatePose(percept.second.pose(), percept.second.lastStamp());
      }
      else
      {
        auto it = entities_.find(percept.second.id());
        if(it == entities_.end())
        {
          BodyPart* new_body_part = nullptr;
          if((percept.second.getType() == BODY_PART_LEFT_HAND) || (percept.second.getType() == BODY_PART_RIGHT_HAND))
            new_body_part = new Hand(percept.second);
          else
            new_body_part = new BodyPart(percept.second);
          it = entities_.insert(std::pair<std::string, BodyPart*>(percept.second.id(), new_body_part)).first;
          if(addToWorld(it->second) == false)
          {
            if(it->second->worldId() != -1)
              addToWorld(it->second, it->second->worldId());
          }
          updateAgent(it->second, AgentType_e::ROBOT);
        }
        if(percept.second.isLocated())
          updateEntityPose(it->second, percept.second.pose(), percept.second.lastStamp());
      }
    }
  }
} // namespace owds