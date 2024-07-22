#ifndef OWDS_AGENTPERCEPTIONMANAGER_H
#define OWDS_AGENTPERCEPTIONMANAGER_H

#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"

namespace owds {
  
  class AgentPerceptionManager : public EntitiesPerceptionManager<BodyPart>
  {
  public:
    explicit AgentPerceptionManager(ros::NodeHandle* nh) : EntitiesPerceptionManager(nh) {}
    ~AgentPerceptionManager();

    std::map<std::string, Agent*> getAgents() const { return agents_; }
    Agent* getAgent(const std::string& id) const 
    { 
      auto it = agents_.find(id);
      if(it == agents_.end())
        return nullptr;
      else
        return it->second;
    }

    std::map<std::string, Sensor*> getEnabledSensors() const { return sensors_register_; }
    std::map<std::string, Sensor*>::iterator createSensor(const std::string& id, const std::string& agent_name = "");

  protected:
    std::map<std::string, Agent*> agents_;
    std::map<std::string, Sensor*> sensors_register_;

    Agent* getAgent(const std::string& agent_name, AgentType_e type);

    std::map<std::string, Agent*>::iterator createAgent(const std::string& name, AgentType_e type);
    
    Agent* updateAgent(BodyPart* body_part, AgentType_e type);
    Agent* updateAgent(Sensor* sensor, AgentType_e type);

    FieldOfView getFov(const std::string& entity_name);
    std::string getOntoValue(const std::vector<std::string>& vect, const std::string& default_value);
    bool getOntoValue(const std::vector<std::string>& vect, bool default_value);
    double getOntoValue(const std::vector<std::string>& vect, double default_value);
  };

} // namespace owds

#endif // OWDS_AGENTPERCEPTIONMANAGER_H