#ifndef OWDS_HUMANSPERCEPTIONMANAGER_H
#define OWDS_HUMANSPERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"
#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Agent.h"

namespace owds {

class HumansPerceptionManager : public EntitiesPerceptionManager<BodyPart>
{
public:
  HumansPerceptionManager(): EntitiesPerceptionManager(){}
  ~HumansPerceptionManager();
  
  Agent* getAgent(const std::string& agent_name);
  std::map<std::string, Agent*> getAgents() { return agents_; }

private:
  std::map<std::string, Agent*> agents_;

  void getPercepts( std::map<std::string, BodyPart>& percepts) override;

  void UpdateAgent(BodyPart* body_part);
};

} // namespace owds

#endif // OWDS_HUMANSPERCEPTIONMANAGER_H