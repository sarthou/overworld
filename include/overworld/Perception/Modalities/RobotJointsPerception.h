#ifndef OWDS_ROBOTJOINTSPERCEPTION_H
#define OWDS_ROBOTJOINTSPERCEPTION_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Perception/PerceptionModuleBase.h"
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>

namespace owds {

class RobotJointsPerception : public owds::PerceptionModuleRosBase<owds::BodyPart, sensor_msgs::JointState>
{
  public:
    /**
     * 
     * @brief Construct a new RobotJointsPerception object
     *
     * @param n
     * @param robotBodyId The Bullet multibody id of the robot (returned after a successful loading of the robot URDF)
     * @param link_to_entity_names The list of link names to create (and update) as Entities in the percepts
     * @param robotWorldClient
     * @param min_period
     */
    RobotJointsPerception(ros::NodeHandle* n,
                        const std::string& robot_name_,
                        const std::vector<std::pair<std::string, BodyPartType_e>>& links_to_entity,
                        BulletClient* robot_world_client,
                        double min_period);
    virtual ~RobotJointsPerception() = default;

    int getRobotBulletId() { return robot_body_id_; }

  protected:
    bool perceptionCallback(const sensor_msgs::JointState& msg) override;
    int robot_body_id_;
    std::string robot_name_;
    std::unordered_map<std::string, int> joint_name_id_;
    std::unordered_map<std::string, int> link_name_id_;
    std::vector<std::pair<std::string, BodyPartType_e>> links_to_entity_;

    double min_period_;

    ros::Time last_update_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    BulletClient* bullet_;

    void loadRobotModel();
};

} // namespace owds

#endif /* OWDS_ROBOTJOINTSPERCEPTION_H */
