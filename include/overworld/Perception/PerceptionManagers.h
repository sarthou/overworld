#ifndef OWDS_PERCEPTIONMANAGER_H
#define OWDS_PERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/AreasPerceptionManager.h"
#include "overworld/Perception/Managers/HumansPerceptionManager.h"
#include "overworld/Perception/Managers/ObjectsPerceptionManager.h"
#include "overworld/Perception/Managers/RobotsPerceptionManager.h"
#include "overworld/Utility/YamlReader.h"

namespace owds {

  class PerceptionManagers
  {
  public:
    explicit PerceptionManagers(ros::NodeHandle* n, BulletClient* bullet_client = nullptr);
    ~PerceptionManagers();

    AreasPerceptionManager areas_manager_;
    RobotsPerceptionManager robots_manager_;
    ObjectsPerceptionManager objects_manager_;
    HumansPerceptionManager humans_manager_;

    void setBulletClient(BulletClient* bullet_client)
    {
      bullet_client_ = bullet_client;
      areas_manager_.setBulletClient(bullet_client_);
      robots_manager_.setBulletClient(bullet_client_);
      objects_manager_.setBulletClient(bullet_client_);
      humans_manager_.setBulletClient(bullet_client_);
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
    BulletClient* bullet_client_;
    std::string robot_name_;
    int robot_bullet_id_;
    Agent* robot_agent_;

    bool applyConfiguration(const std::string& config_path, YamlElement& modules_list, bool display = true);

    YamlReader configuration_;
  };

} // namespace owds

#endif // OWDS_PERCEPTIONMANAGER_H