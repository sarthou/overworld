#ifndef OWDS_HUMANSPERCEPTIONMANAGER_H
#define OWDS_HUMANSPERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/AgentPerceptionManager.h"

namespace owds {

  class HumansPerceptionManager : public AgentPerceptionManager
  {
  public:
    explicit HumansPerceptionManager(ros::NodeHandle* nh) : AgentPerceptionManager(nh) {}

    Agent* getAgent(const std::string& agent_name) { return AgentPerceptionManager::getAgent(agent_name, AgentType_e::HUMAN); }

  private:
    DataFusionBase fusioner_;

    void getPercepts(const std::string& module_name, std::map<std::string, Percept<BodyPart>>& percepts) override;
    void reasoningOnUpdate() override;

    void fromfusedToEntities();
  };

} // namespace owds

#endif // OWDS_HUMANSPERCEPTIONMANAGER_H