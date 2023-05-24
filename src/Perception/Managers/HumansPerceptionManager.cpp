#include "overworld/Perception/Managers/HumansPerceptionManager.h"

#include "overworld/Utility/ShellDisplay.h"
#include "overworld/BasicTypes/Hand.h"

namespace owds {

void HumansPerceptionManager::getPercepts( std::map<std::string, BodyPart>& percepts)
{
    for(auto& percept : percepts)
    {
        if(percept.second.isLocated())
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
              UpdateAgent(it->second, AgentType_e::HUMAN);
          }
          
          updateEntityPose(it->second, percept.second.pose(), percept.second.lastStamp());
        }
    }
}

} // namespace owds