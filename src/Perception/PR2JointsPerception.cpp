#include "overworld/Perception/PR2JointsPerception.h"

namespace owds {

PR2JointsPerception::PR2JointsPerception(ros::NodeHandle* n, int robot_body_id, std::vector<std::string> link_to_entity_names,
                                         BulletClient* robot_world_client)
    : PerceptionModuleRosBase(n, "/joint_states"), robot_body_id_(robot_body_id), link_to_entity_names_(link_to_entity_names),
      bullet_(robot_world_client)
{
    auto p = bullet_->findJointAndLinkIndices(robot_body_id_);
    joint_name_id_ = p.first;
    link_name_id_ = p.second;
    for (const auto& name : link_to_entity_names_)
    {
        percepts_.emplace(name, name);
        if (link_name_id_.count(name) == 0){
            std::cout << "Error: link name '" << name << "' passed as 'link_to_entity_names' of ctor of PR2JointsPerception does not exist in Bullet.";
            throw std::runtime_error("Link name not found in Bullet.");
        }
    }
}

bool PR2JointsPerception::perceptionCallback(const sensor_msgs::JointState& msg)
{
    for (size_t i = 0; i < msg.name.size(); i++)
    {
        std::string name = msg.name[i];
        if (joint_name_id_.count(name) != 1)
        {
            std::cout << "Joint name not found in Bullet: " << name << std::endl;
            continue;
        }
        bullet_->resetJointState(robot_body_id_, joint_name_id_[name], msg.position[i]);
    }
    for (const auto& name : link_to_entity_names_)
    {
        b3LinkState link = bullet_->getLinkState(robot_body_id_, link_name_id_[name]);
        double* pos = link.m_worldLinkFramePosition;
        double* rot = link.m_worldLinkFrameOrientation;
        percepts_.at(name).updatePose({{pos[0], pos[1], pos[2]}}, {{rot[0], rot[1], rot[2], rot[3]}}, msg.header.stamp);
    }
    return true;
}
} // namespace owds