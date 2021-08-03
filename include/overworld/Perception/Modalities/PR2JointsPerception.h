#ifndef OWDS_PR2JOINTSPERCEPTION_H
#define OWDS_PR2JOINTSPERCEPTION_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Perception/PerceptionModuleBase.h"
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>

namespace owds {

class PR2JointsPerception : public owds::PerceptionModuleRosBase<owds::BodyPart, sensor_msgs::JointState>
{
  public:
    /**
     * 
     * @brief Construct a new PR2JointsPerception object
     *
     * @param n
     * @param robotBodyId The Bullet multibody id of the robot (returned after a successful loading of the robot URDF)
     * @param link_to_entity_names The list of link names to create (and update) as Entities in the percepts
     * @param robotWorldClient
     * @param min_period
     */
    PR2JointsPerception(ros::NodeHandle* n,
                        int robotBodyId,
                        const std::string& robot_name_,
                        const std::vector<std::pair<std::string, BodyPartType_e>>& links_to_entity,
                        BulletClient* robot_world_client,
                        double min_period);
    virtual ~PR2JointsPerception() = default;

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
};

} // namespace owds

#endif /* OWDS_PR2JOINTSPERCEPTION_H */
