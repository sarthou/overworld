#include "overworld/Perception/PR2JointsPerception.h"

namespace owds {

PR2JointsPerception::PR2JointsPerception(ros::NodeHandle* n, int robotBodyId) : PerceptionModuleRosBase(n, "/joint_states"),
                                                                                robot_body_id_(robotBodyId)
{
    bullet_ = PhysicsServers::connectPhysicsServer(CONNECT_SHARED_MEMORY); // TODO: unsure, which connection method should I use?
    joint_name_id_ = bullet_->findJointIndices(robot_body_id_);
}

bool PR2JointsPerception::perceptionCallback(const sensor_msgs::JointState& msg)
{
    for (size_t i = 0; i < msg.name.size(); i++)
    {
        std::string name = msg.name[i];
        if (joint_name_id_.count(name) != 1){
            std::cout << "Joint name not found in Bullet: " << name << std::endl;
            continue;
        }
        bullet_->resetJointState(robot_body_id_, joint_name_id_[name], msg.position[i]);
    }
    return true;
}
} // namespace owds