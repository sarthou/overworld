#ifndef OWDS_ROBOTSPERCEPTIONMANAGER_H
#define OWDS_ROBOTSPERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/AgentPerceptionManager.h"

namespace owds {

  class RobotsPerceptionManager : public AgentPerceptionManager
  {
  public:
    RobotsPerceptionManager() = default;
    virtual ~RobotsPerceptionManager() = default;

    Agent* getAgent(const std::string& agent_name) { return AgentPerceptionManager::getAgent(agent_name, AgentType_e::ROBOT); }

  private:
    void getPercepts(const std::string& module_name, std::map<std::string, Percept<BodyPart>>& percepts) override;
  };

} // namespace owds

#endif // OWDS_ROBOTSPERCEPTIONMANAGER_H