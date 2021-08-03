#include "overworld/Perception/Modalities/PR2JointsPerception.h"

namespace owds {

PR2JointsPerception::PR2JointsPerception(ros::NodeHandle* n, int robot_body_id, std::vector<std::string> link_to_entity_names,
                                         BulletClient* robot_world_client, double min_period)
    : PerceptionModuleRosBase(n, "/joint_states"), robot_body_id_(robot_body_id), link_to_entity_names_(link_to_entity_names),
      tf2_listener_(tf_buffer_), bullet_(robot_world_client), min_period_(min_period)
{
    auto p = bullet_->findJointAndLinkIndices(robot_body_id_);
    joint_name_id_ = p.first;
    link_name_id_ = p.second;
    for (const auto& name : link_to_entity_names_)
    {
        percepts_.emplace(name, name);
        percepts_.at(name).setAgentName("pr2_robot");
        if (link_name_id_.count(name) == 0)
        {
            std::cout << "Error: link name '" << name
                      << "' passed as 'link_to_entity_names' of ctor of PR2JointsPerception does not exist in Bullet.";
            throw std::runtime_error("Link name '" + name + "' not found in Bullet.");
        }
    }
    percepts_.emplace("base_footprint", BodyPart("base_footprint"));
    percepts_.at("base_footprint").setAgentName("pr2_robot");
}

bool PR2JointsPerception::perceptionCallback(const sensor_msgs::JointState& msg)
{
    if ((ros::Time::now() - last_update_).toSec() < min_period_){
        return false;
    }
    geometry_msgs::TransformStamped robot_base = tf_buffer_.lookupTransform("map", "base_footprint", msg.header.stamp, ros::Duration(1.0));
    bullet_->resetBasePositionAndOrientation(
        robot_body_id_, {robot_base.transform.translation.x, robot_base.transform.translation.y, robot_base.transform.translation.z},
        {robot_base.transform.rotation.x, robot_base.transform.rotation.y, robot_base.transform.rotation.z, robot_base.transform.rotation.w});
    percepts_.at("base_footprint")
        .updatePose(
            {robot_base.transform.translation.x, robot_base.transform.translation.y, robot_base.transform.translation.z},
            {robot_base.transform.rotation.x, robot_base.transform.rotation.y, robot_base.transform.rotation.z, robot_base.transform.rotation.w});
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
    last_update_ = ros::Time::now();
    return true;
}
} // namespace owds