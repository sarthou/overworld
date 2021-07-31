#ifndef OWDS_PR2JOINTSPERCEPTION_H
#define OWDS_PR2JOINTSPERCEPTION_H

#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Perception/PerceptionModuleBase.h"
#include <sensor_msgs/JointState.h>

namespace owds {

class PR2JointsPerception : public owds::PerceptionModuleRosBase<owds::Entity, sensor_msgs::JointState>
{
  public:
    PR2JointsPerception(ros::NodeHandle* n, int robotBodyId);
    virtual ~PR2JointsPerception() = default;

  protected:
    bool perceptionCallback(const sensor_msgs::JointState& msg) override;
    int robot_body_id_;
    std::unordered_map<std::string, int> joint_name_id_;

    BulletClient* bullet_;
};

} // namespace owds

#endif /* OWDS_PR2JOINTSPERCEPTION_H */
