#ifndef OWDS_PR2JOINTSPERCEPTION_H
#define OWDS_PR2JOINTSPERCEPTION_H

#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Perception/PerceptionModuleBase.h"
#include <sensor_msgs/JointState.h>
#include "overworld/BasicTypes/BodyPart.h"

namespace owds {

class PR2JointsPerception : public owds::PerceptionModuleRosBase<owds::Entity, sensor_msgs::JointState>
{
  public:
  /**
   * @brief Construct a new PR2JointsPerception object
   * 
   * @param n 
   * @param robotBodyId The Bullet multibody id of the robot (returned after a successful loading of the robot URDF)
   * @param link_to_entity_names The list of link names to create (and update) as Entities in the percepts
   * @param robotWorldClient 
   */
    PR2JointsPerception(ros::NodeHandle* n, int robotBodyId, std::vector<std::string> link_to_entity_names, BulletClient* robotWorldClient);
    virtual ~PR2JointsPerception() = default;

  protected:
    bool perceptionCallback(const sensor_msgs::JointState& msg) override;
    int robot_body_id_;
    std::unordered_map<std::string, int> joint_name_id_;
    std::unordered_map<std::string, int> link_name_id_;
    std::vector<std::string> link_to_entity_names_;

    BulletClient* bullet_;
};

} // namespace owds

#endif /* OWDS_PR2JOINTSPERCEPTION_H */
