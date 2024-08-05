#include "overworld/Perception/Managers/HumansPerceptionManager.h"

#include "overworld/BasicTypes/Hand.h"
#include "overworld/Utility/ShellDisplay.h"

namespace owds {

  void HumansPerceptionManager::getPercepts(const std::string& module_name, std::map<std::string, Percept<BodyPart>>& percepts)
  {        
    for(auto& percept : percepts)
    {
      percept.second.setModuleName(module_name);
      if(percept.second.getSensorId().empty() == false) // we avoid problems with static object
      {
        auto* sensor = getAgent(percept.second.getAgentName())->getSensor(percept.second.getSensorId());
        if(sensor != nullptr)
          sensor->setPerceptseen(percept.first);
      }

      std::string part_id = percept.first;
      if(percept.second.isLocated()) // TODO consider human disparition
        fusionAggregated(part_id, module_name, percept.second);
    }
  }

  void HumansPerceptionManager::fromfusedToEntities()
  {
    for(auto& percept : fusioned_percepts_)
    {
      auto it = entities_.find(percept.second->id());
      if(it == entities_.end())
      {
        if(!percept.second->getSensorId().empty())
          fusionRegister(percept.first, percept.second->getSensorId(), percept.second->getModuleName());

        if(percept.second->isLocated() == false)
          continue;

        BodyPart* new_body_part = nullptr;
        if((percept.second->getType() == BODY_PART_LEFT_HAND) || (percept.second->getType() == BODY_PART_RIGHT_HAND))
          new_body_part = new Hand(*percept.second);
        else
          new_body_part = new BodyPart(*percept.second);
        it = entities_.emplace(std::pair<std::string, BodyPart*>(percept.second->id(), new_body_part)).first;
        if(addToBullet(it->second) == false)
        {
          if(it->second->bulletId() != -1)
            addToBullet(it->second, it->second->bulletId());
        }
        updateAgent(it->second, AgentType_e::HUMAN);
      }

      updateEntityPose(it->second, percept.second->pose(), percept.second->lastStamp());

      if(percept.second->getType() == BODY_PART_HEAD)
      {
        auto* agent = getAgent(percept.second->getAgentName());
        if(agent->getSensors().empty()) // a human has only one sensor
        {
          auto human_eyes = onto_->individuals.getOn(percept.first, "hasSensor");
          if(onto_->individuals.getErrorCode() != 0) // when the program is shut down
            return;

          for(auto& eye : human_eyes)
          {
            auto inner_it = sensors_register_.find(eye);
            if(inner_it == sensors_register_.end())
            {
              inner_it = createSensor(eye, percept.second->getAgentName());
              updateAgent(inner_it->second, AgentType_e::HUMAN);
            }
            if(percept.second->isLocated())
              inner_it->second->updatePose(percept.second->pose(), percept.second->lastStamp());
          }
        }
      }
    }
  }

  void HumansPerceptionManager::reasoningOnUpdate()
  {
    fusioner_.fuseData(fusioned_percepts_, aggregated_);
    fromfusedToEntities();

    for(auto& percept : aggregated_)
      percept.second.clear();
  }

} // namespace owds
