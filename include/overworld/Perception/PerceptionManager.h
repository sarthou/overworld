#ifndef OWDS_PERCEPTIONMANAGER_H
#define OWDS_PERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/RobotsPerceptionManager.h"
#include "overworld/Perception/Managers/ObjectsPerceptionManager.h"
#include "overworld/Perception/Managers/HumansPerceptionManager.h"

#include "overworld/Perception/PerceptionConfiguration.h"

namespace owds {

class PerceptionManager
{
public:
    explicit PerceptionManager(ros::NodeHandle* n, BulletClient* bullet_client = nullptr);
    ~PerceptionManager();

    RobotsPerceptionManager robots_manager_;
    ObjectsPerceptionManager objects_manager_;
    HumansPerceptionManager humans_manager_;

    void setBulletClient(BulletClient* bullet_client)
    {
        bullet_client_ = bullet_client;
        robots_manager_.setBulletClient(bullet_client_);
        objects_manager_.setBulletClient(bullet_client_);
        humans_manager_.setBulletClient(bullet_client_);
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

    bool applyConfiguration(const std::string& config_path, ConfigElement& modules_list, bool display = true);

    PerceptionConfiguration configuration_;
};

} // namespace owds

#endif // OWDS_PERCEPTIONMANAGER_H