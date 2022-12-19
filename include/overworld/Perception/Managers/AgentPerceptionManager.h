#ifndef OWDS_AGENTPERCEPTIONMANAGER_H
#define OWDS_AGENTPERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"
#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Agent.h"

namespace owds {

class AgentPerceptionManager : public EntitiesPerceptionManager<BodyPart>
{
public:
  AgentPerceptionManager() : EntitiesPerceptionManager(){}
  ~AgentPerceptionManager();

  std::map<std::string, Agent*> getAgents() const { return agents_; }

protected:
  std::map<std::string, Agent*> agents_;

  Agent* getAgent(const std::string& agent_name, AgentType_e type);

  std::map<std::string, Agent*>::iterator createAgent(const std::string& name, AgentType_e type);
  void UpdateAgent(BodyPart* body_part, AgentType_e type);
};

} // namespace owds

#endif // OWDS_AGENTPERCEPTIONMANAGER_H