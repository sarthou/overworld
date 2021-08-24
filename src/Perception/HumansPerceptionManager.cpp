#include "overworld/Perception/HumansPerceptionManager.h"

#include "overworld/Utility/ShellDisplay.h"
#include "overworld/BasicTypes/Hand.h"

namespace owds {

HumansPerceptionManager::~HumansPerceptionManager()
{
  for(auto agent : agents_)
    delete agent.second;
  agents_.clear();
}

Agent* HumansPerceptionManager::getAgent(const std::string& agent_name)
{
  auto it = agents_.find(agent_name);
  if(it == agents_.end())
  {
    auto agent = new Agent(agent_name, FieldOfView(60.0, 80.0,0.1,5), AgentType_e::HUMAN);
    it = agents_.insert(std::pair<std::string, Agent*>(agent_name, agent)).first;
  }

  return it->second;
}

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
              addToBullet(it->second);
              UpdateAgent(it->second);
          }
          
          updateEntityPose(it->second, percept.second.pose(), percept.second.lastStamp());
        }
    }
}

void HumansPerceptionManager::UpdateAgent(BodyPart* body_part)
{
  if(body_part->isAgentKnown())
  {
    auto it_agent = agents_.find(body_part->getAgentName());
    if(it_agent == agents_.end())
    {
      auto agent = new Agent(body_part->getAgentName(), FieldOfView(60.0, 80.0,0.1,5), AgentType_e::HUMAN);
      it_agent = agents_.insert(std::pair<std::string, Agent*>(body_part->getAgentName(), agent)).first;
    }

    switch (body_part->getType())
    {    
      case BODY_PART_HEAD:
      {
        it_agent->second->setHead(body_part);
        ShellDisplay::info("Head has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_LEFT_HAND:
      {
        it_agent->second->setLeftHand(static_cast<Hand*>(body_part));
        ShellDisplay::info("Left hand has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_RIGHT_HAND:
      {
        it_agent->second->setRightHand(static_cast<Hand*>(body_part));
        ShellDisplay::info("Right hand has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_TORSO:
      {
        it_agent->second->setTorso(body_part);
        ShellDisplay::info("Torso has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_BASE:
      {
        it_agent->second->setBase(body_part);
        ShellDisplay::info("Base has been setted for " + it_agent->second->getId());
        break;
      }
    }
  }
  else
    ShellDisplay::warning("The agent of the body part " + body_part->id() + " is undefined");
}

} // namespace owds