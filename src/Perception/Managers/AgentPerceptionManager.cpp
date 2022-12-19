#include "overworld/Perception/Managers/AgentPerceptionManager.h"

#include "overworld/Utility/ShellDisplay.h"
#include "overworld/BasicTypes/Hand.h"

namespace owds {

AgentPerceptionManager::~AgentPerceptionManager()
{
  for(auto agent : agents_)
    delete agent.second;
  agents_.clear();
}

Agent* AgentPerceptionManager::getAgent(const std::string& agent_name, AgentType_e type)
{
  auto it = agents_.find(agent_name);
  if(it == agents_.end())
    it = createAgent(agent_name, type);

  return it->second;
}

std::map<std::string, Agent*>::iterator AgentPerceptionManager::createAgent(const std::string& name, AgentType_e type)
{
  std::map<std::string, Agent*>::iterator it;
  auto agent = new Agent(name, FieldOfView(60, 80, 0.1, 12), type);
  it = agents_.insert(std::pair<std::string, Agent*>(name, agent)).first;

  if(type == AgentType_e::ROBOT)
    ShellDisplay::info("[AgentPerceptionManager] Robot " + name + " has been created with FoV: \n" + agent->getFieldOfView().toString());
  else
    ShellDisplay::info("[AgentPerceptionManager] Human " + name + " has been created with FoV: \n" + agent->getFieldOfView().toString());

  return it;
}

void AgentPerceptionManager::UpdateAgent(BodyPart* body_part, AgentType_e type)
{
  if(body_part->isAgentKnown())
  {
    auto it_agent = agents_.find(body_part->getAgentName());
    if(it_agent == agents_.end())
      it_agent = createAgent(body_part->getAgentName(), type);

    switch (body_part->getType())
    {    
      case BODY_PART_HEAD:
      {
        it_agent->second->setHead(body_part);
        ShellDisplay::info("[AgentPerceptionManager] Head has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_LEFT_HAND:
      {
        it_agent->second->setLeftHand(static_cast<Hand*>(body_part));
        ShellDisplay::info("[AgentPerceptionManager] Left hand has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_RIGHT_HAND:
      {
        it_agent->second->setRightHand(static_cast<Hand*>(body_part));
        ShellDisplay::info("[AgentPerceptionManager] Right hand has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_TORSO:
      {
        it_agent->second->setTorso(body_part);
        ShellDisplay::info("[AgentPerceptionManager] Torso has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_BASE:
      {
        it_agent->second->setBase(body_part);
        ShellDisplay::info("[AgentPerceptionManager] Base has been setted for " + it_agent->second->getId());
        break;
      }
    }
  }
  else
    ShellDisplay::warning("[AgentPerceptionManager] The agent of the body part " + body_part->id() + " is undefined");
}

} // namespace owds