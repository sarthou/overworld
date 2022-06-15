#ifndef OWDS_ROBOTSPERCEPTIONMANAGER_H
#define OWDS_ROBOTSPERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"
#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Agent.h"

namespace owds {

class RobotsPerceptionManager : public EntitiesPerceptionManager<BodyPart>
{
public:
  RobotsPerceptionManager(): EntitiesPerceptionManager(){}
  ~RobotsPerceptionManager();

  Agent* getAgent(const std::string& agent_name);
  std::map<std::string, Agent*> getAgents() const { return agents_; }

private:
  std::map<std::string, Agent*> agents_;

  void getPercepts( std::map<std::string, BodyPart>& percepts) override;

  void UpdateAgent(BodyPart* body_part);
};

} // namespace owds

#endif // OWDS_ROBOTSPERCEPTIONMANAGER_H