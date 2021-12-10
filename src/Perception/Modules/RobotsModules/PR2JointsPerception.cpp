#include "overworld/Perception/Modules/RobotsModules/PR2JointsPerception.h"

#include <ros/package.h>

namespace owds {

PR2JointsPerception::PR2JointsPerception(ros::NodeHandle* n,
                                         const std::string& robot_name,
                                         const std::vector<std::pair<std::string, BodyPartType_e>>& links_to_entity,
                                         BulletClient* robot_world_client,
                                         double min_period)
                                                            : PerceptionModuleRosBase(n, "/joint_states"),
                                                            robot_name_(robot_name),
                                                            links_to_entity_(links_to_entity),
                                                            tf2_listener_(tf_buffer_),
                                                            bullet_(robot_world_client),
                                                            min_period_(min_period)
{
    loadPr2Model();

    auto p = bullet_->findJointAndLinkIndices(robot_body_id_);
    joint_name_id_ = p.first;
    link_name_id_ = p.second;
    for (const auto& link_pair : links_to_entity_)
    {
        percepts_.emplace(link_pair.first, link_pair.first);
        percepts_.at(link_pair.first).setAgentName(robot_name_);
        percepts_.at(link_pair.first).setType(link_pair.second);
        if (link_name_id_.count(link_pair.first) == 0)
        {
            std::cout << "Error: link name '" << link_pair.first
                      << "' passed as 'link_to_entity_names' of ctor of PR2JointsPerception does not exist in Bullet.";
            throw std::runtime_error("Link name '" + link_pair.first + "' not found in Bullet.");
        }
    }
    percepts_.emplace("base_footprint", BodyPart("base_footprint"));
    percepts_.at("base_footprint").setAgentName(robot_name_);
    percepts_.at("base_footprint").setType(BODY_PART_BASE);
}

bool PR2JointsPerception::perceptionCallback(const sensor_msgs::JointState& msg)
{
    if ((ros::Time::now() - last_update_).toSec() < min_period_){
        return false;
    }
    geometry_msgs::TransformStamped robot_base;
    try {
        robot_base = tf_buffer_.lookupTransform("map", "base_footprint", msg.header.stamp, ros::Duration(1.0));
    }
    catch(...) {
        return false;
    }
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
            //std::cout << "Joint name not found in Bullet: " << name << std::endl;
            continue;
        }
        bullet_->resetJointState(robot_body_id_, joint_name_id_[name], msg.position[i]);
    }
    for (const auto& link_pair : links_to_entity_)
    {
        b3LinkState link = bullet_->getLinkState(robot_body_id_, link_name_id_[link_pair.first]);
        double* pos = link.m_worldLinkFramePosition;
        double* rot = link.m_worldLinkFrameOrientation;
        percepts_.at(link_pair.first).updatePose({{pos[0], pos[1], pos[2]}}, {{rot[0], rot[1], rot[2], rot[3]}}, msg.header.stamp);
    }
    last_update_ = ros::Time::now();
    return true;
}

void PR2JointsPerception::loadPr2Model()
{
    std::string path_pr2_description = ros::package::getPath("pr2_description");
	path_pr2_description = path_pr2_description.substr(0, path_pr2_description.size() - std::string("/pr2_description").size());
	std::string path_overworld = ros::package::getPath("overworld");
	
	bullet_->setAdditionalSearchPath(path_overworld + "/models");

    std::string urdf = n_->param<std::string>("/robot_description", "");
    if (urdf == "")
    {
	    robot_body_id_ = bullet_->loadURDF("pr2.urdf", {0,0,0}, {0,0,0,1});
    }else{
        robot_body_id_ = bullet_->loadURDFRaw(urdf, "pr2_tmp.urdf", {0,0,0}, {0,0,0,1});
    }
}

} // namespace owds