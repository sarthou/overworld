#ifndef OWDS_HUMANSPERCEPTIONMANAGER_H
#define OWDS_HUMANSPERCEPTIONMANAGER_H

#include "overworld/Perception/EntitiesPerceptionManager.h"
#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Agent.h"

namespace owds {

class HumansPerceptionManager : public EntitiesPerceptionManager<BodyPart>
{
public:
  inline HumansPerceptionManager(): EntitiesPerceptionManager(){}
  Agent* getAgent(const std::string& agent_name);

private:
  std::map<std::string, Agent> agents_;

  void getPercepts(const std::map<std::string, BodyPart>& percepts) override;

  void UpdateAgent(BodyPart* body_part);
};

} // namespace owds

#endif // OWDS_HUMANSPERCEPTIONMANAGER_H