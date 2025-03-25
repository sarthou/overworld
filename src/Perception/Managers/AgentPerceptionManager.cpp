#include "overworld/Perception/Managers/AgentPerceptionManager.h"

#include <string>
#include <cstddef>

#include "overworld/BasicTypes/Hand.h"
#include "overworld/Utils/ShellDisplay.h"

namespace owds {

  AgentPerceptionManager::~AgentPerceptionManager()
  {
    for(auto agent : agents_)
      delete agent.second;

    for(auto sensor : sensors_register_)
      delete sensor.second;

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
    auto* agent = new Agent(name, type);
    it = agents_.emplace(name, agent).first;

    if(type == AgentType_e::ROBOT)
      ShellDisplay::info("[AgentPerceptionManager] Robot " + name + " has been created");
    else
      ShellDisplay::info("[AgentPerceptionManager] Human " + name + " has been created");

    return it;
  }

  Agent* AgentPerceptionManager::updateAgent(BodyPart* body_part, AgentType_e type)
  {
    if(body_part->isAgentKnown())
    {
      auto it_agent = agents_.find(body_part->getAgentName());
      if(it_agent == agents_.end())
        it_agent = createAgent(body_part->getAgentName(), type);

      switch(body_part->getType())
      {
      case BODY_PART_HEAD:
      {
        it_agent->second->setHead(body_part);
        ShellDisplay::info("[AgentPerceptionManager] Head " + body_part->id() + " has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_LEFT_HAND:
      {
        it_agent->second->setLeftHand(static_cast<Hand*>(body_part));
        ShellDisplay::info("[AgentPerceptionManager] Left hand " + body_part->id() + " has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_RIGHT_HAND:
      {
        it_agent->second->setRightHand(static_cast<Hand*>(body_part));
        ShellDisplay::info("[AgentPerceptionManager] Right hand " + body_part->id() + " has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_TORSO:
      {
        it_agent->second->setTorso(body_part);
        ShellDisplay::info("[AgentPerceptionManager] Torso " + body_part->id() + " has been setted for " + it_agent->second->getId());
        break;
      }
      case BODY_PART_BASE:
      {
        it_agent->second->setBase(body_part);
        ShellDisplay::info("[AgentPerceptionManager] Base " + body_part->id() + " has been setted for " + it_agent->second->getId());
        break;
      }
      default:
        return nullptr;
      }
      return it_agent->second;
    }
    else
    {
      ShellDisplay::warning("[AgentPerceptionManager] The agent of the body part " + body_part->id() + " is undefined");
      return nullptr;
    }
  }

  Agent* AgentPerceptionManager::updateAgent(Sensor* sensor, AgentType_e type)
  {
    if(sensor->isAgentKnown())
    {
      auto it_agent = agents_.find(sensor->getAgentName());
      if(it_agent == agents_.end())
        it_agent = createAgent(sensor->getAgentName(), type);
      it_agent->second->setSensor(sensor);
      ShellDisplay::info("[AgentPerceptionManager] Sensor " + sensor->id() + " has been setted for " + it_agent->second->getId() +
                         " with FoV " + sensor->getFieldOfView().toString());
      return it_agent->second;
    }
    else
    {
      ShellDisplay::warning("[AgentPerceptionManager] The agent of the sensor " + sensor->id() + " is undefined");
      return nullptr;
    }
  }

  std::map<std::string, Sensor*>::iterator AgentPerceptionManager::createSensor(const std::string& id, const std::string& agent_name)
  {
    const std::string& sensor_id = id;
    std::vector<std::string> onto_res;
    onto_res = onto_->individuals.getOn(sensor_id, "hasFrameId");
    std::string frame_id = getOntoValue(onto_res, std::string(""));
    onto_res = onto_->individuals.getOn(sensor_id, "isStatic");
    bool is_static = getOntoValue(onto_res, true);

    auto* sensor = new Sensor(sensor_id, frame_id, is_static, getFov(sensor_id));
    if(!agent_name.empty())
      sensor->setAgentName(agent_name);
    auto it = sensors_register_.emplace(sensor_id, sensor).first;
    frames_register_.emplace(frame_id, sensor);

    return it;
  }

  FieldOfView AgentPerceptionManager::getFov(const std::string& entity_name)
  {
    auto entity_fov = onto_->individuals.getOn(entity_name, "hasFieldOfView");
    if(entity_fov.size())
    {
      std::vector<std::string> onto_res;
      onto_res = onto_->individuals.getOn(entity_fov.front(), "fovHasClipNear");
      double clip_near = getOntoValue(onto_res, 0.1);
      onto_res = onto_->individuals.getOn(entity_fov.front(), "fovHasClipFar");
      double clip_far = getOntoValue(onto_res, 12.0);
      onto_res = onto_->individuals.getOn(entity_fov.front(), "fovHasHeight");
      double height = getOntoValue(onto_res, 60.0);
      onto_res = onto_->individuals.getOn(entity_fov.front(), "fovHasWidth");
      double width = getOntoValue(onto_res, 80.0);

      return FieldOfView(height, width, clip_near, clip_far);
    }
    else
      return FieldOfView(60.0, 80.0, 0.1, 12.0);
  }

  std::string AgentPerceptionManager::getOntoValue(const std::vector<std::string>& vect, const std::string& default_value)
  {
    if(!vect.empty())
    {
      std::string str_value = vect.front();
      size_t pose = str_value.find("#");
      if(pose != std::string::npos)
        str_value = str_value.substr(pose + 1);

      return str_value;
    }
    else
      return default_value;
  }

  bool AgentPerceptionManager::getOntoValue(const std::vector<std::string>& vect, bool default_value)
  {
    if(!vect.empty())
    {
      std::string str_value = vect.front();
      size_t pose = str_value.find("#");
      if(pose != std::string::npos)
        str_value = str_value.substr(pose + 1);

      std::transform(str_value.begin(), str_value.end(), str_value.begin(), ::tolower);
      if(str_value == "true" || str_value == "1")
        return true;
      else if(str_value == "false" || str_value == "0")
        return false;

      ShellDisplay::warning("[EntitiesPerceptionManager] " + str_value + " cannot be converted to bool. Use default value " + std::to_string(default_value) + " instead");
      return default_value;
    }
    else
      return default_value;
  }

  double AgentPerceptionManager::getOntoValue(const std::vector<std::string>& vect, double default_value)
  {
    if(!vect.empty())
    {
      std::string str_value = vect.front();
      size_t pose = str_value.find("#");
      if(pose != std::string::npos)
        str_value = str_value.substr(pose + 1);

      try
      {
        return std::stod(str_value);
      }
      catch(const std::exception& e)
      {
        ShellDisplay::warning("[EntitiesPerceptionManager] " + str_value + " cannot be converted to double. Use default value " + std::to_string(default_value) + " instead");
        return default_value;
      }
    }
    else
      return default_value;
  }

} // namespace owds