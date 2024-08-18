#include "overworld/Perception/Managers/RobotsPerceptionManager.h"

#include "overworld/BasicTypes/Hand.h"
#include "overworld/Utils/ShellDisplay.h"

namespace owds {

  void RobotsPerceptionManager::getPercepts(const std::string& module_name, std::map<std::string, Percept<BodyPart>>& percepts)
  {
    for(auto& percept : percepts)
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
        if(addToBullet(it->second) == false)
        {
          if(it->second->bulletId() != -1)
            addToBullet(it->second, it->second->bulletId());
        }
        updateAgent(it->second, AgentType_e::ROBOT);
      }

      if(percept.second.isLocated())
        updateEntityPose(it->second, percept.second.pose(), percept.second.lastStamp());
    }
  }

} // namespace owds