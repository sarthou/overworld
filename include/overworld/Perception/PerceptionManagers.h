#ifndef OWDS_PERCEPTIONMANAGER_H
#define OWDS_PERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/AreasPerceptionManager.h"
#include "overworld/Perception/Managers/HumansPerceptionManager.h"
#include "overworld/Perception/Managers/ObjectsPerceptionManager.h"
#include "overworld/Perception/Managers/RobotsPerceptionManager.h"
#include "overworld/Utils/YamlReader.h"
#include "overworld/Engine/Engine.h"

namespace owds {

  class PerceptionManagers
  {
  public:
    explicit PerceptionManagers(ros::NodeHandle* n, WorldEngine* world_client = nullptr);
    ~PerceptionManagers();

    AreasPerceptionManager areas_manager_;
    RobotsPerceptionManager robots_manager_;
    ObjectsPerceptionManager objects_manager_;
    HumansPerceptionManager humans_manager_;

    void setWorldClient(WorldEngine* world_client)
    {
      world_client_ = world_client;
      areas_manager_.setWorldClient(world_client_);
      robots_manager_.setWorldClient(world_client_);
      objects_manager_.setWorldClient(world_client_);
      humans_manager_.setWorldClient(world_client_);
    }

    void setOwnerAgentName(const std::string& agent_name)
    {
      robots_manager_.setOwnerAgentName(agent_name);
      objects_manager_.setOwnerAgentName(agent_name);
      humans_manager_.setOwnerAgentName(agent_name);
    }

    void update();

    bool applyConfigurationRobot(const std::string& config_path);
    bool applyConfigurationHuman(const std::string& config_path);

    std::string getRobotName() { return robot_name_; }

  private:
    ros::NodeHandle* n_;
    WorldEngine* world_client_;
    std::string robot_name_;
    int robot_engine_id_;
    Agent* robot_agent_;

    bool applyConfiguration(const std::string& config_path, YamlElement& modules_list, bool display = true);

    YamlReader configuration_;
  };

} // namespace owds

#endif // OWDS_PERCEPTIONMANAGER_H