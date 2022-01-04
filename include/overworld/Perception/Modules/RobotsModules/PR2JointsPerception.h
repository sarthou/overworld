#ifndef OWDS_PR2JOINTSPERCEPTION_H
#define OWDS_PR2JOINTSPERCEPTION_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>

namespace owds {

class PR2JointsPerception : public owds::PerceptionModuleRosBase<owds::BodyPart, sensor_msgs::JointState>
{
  public:
    PR2JointsPerception();
    virtual ~PR2JointsPerception() = default;

    void setParameter(const std::string& parameter_name, const std::string& parameter_value);
    bool closeInitialization();

    std::string getAgentName() { return robot_name_; } 
    int getAgentBulletId() { return robot_bullet_id_; }

  protected:
    bool perceptionCallback(const sensor_msgs::JointState& msg) override;
    std::string robot_name_;

    std::string right_hand_link_;
    std::string left_hand_link_;
    std::string head_ink_;

    std::unordered_map<std::string, int> joint_name_id_;
    std::unordered_map<std::string, int> links_name_id_;
    std::vector<std::pair<std::string, BodyPartType_e>> links_to_entity_;

    double min_period_;

    ros::Time last_update_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    void loadPr2Model();
};

} // namespace owds

#endif /* OWDS_PR2JOINTSPERCEPTION_H */
