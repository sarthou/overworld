#include "overworld/Perception/Modules/RobotsModules/PR2JointsPerception.h"

#include <ros/package.h>

#include <pluginlib/class_list_macros.h>

namespace owds {

PR2JointsPerception::PR2JointsPerception(): PerceptionModuleRosBase("/joint_states"),
                                            tf2_listener_(tf_buffer_)
{
    min_period_ = 0.9;
}

void PR2JointsPerception::setParameter(const std::string& parameter_name, const std::string& parameter_value)
{
    if(parameter_name == "name")
        robot_name_ = parameter_value;
    else if(parameter_name == "min_period")
        min_period_ = std::stod(parameter_value);
    else if(parameter_name == "right_hand")
        right_hand_link_ = parameter_value;
    else if(parameter_name == "left_hand")
        left_hand_link_ = parameter_value;
    else if(parameter_name == "head")
        head_ink_ = parameter_value;
    else
        ShellDisplay::warning("[PR2JointsPerception] Unkown parameter " + parameter_name);
}

bool PR2JointsPerception::closeInitialization()
{
    if(robot_name_ == "")
    {
        ShellDisplay::error("[PR2JointsPerception] No robot name has been defined");
        return false;
    }
    if(right_hand_link_ == "")
    {
        ShellDisplay::error("[PR2JointsPerception] No right hand link has been defined");
        return false;
    }
    if(left_hand_link_ == "")
    {
        ShellDisplay::error("[PR2JointsPerception] No left hand link has been defined");
        return false;
    }
    if(head_ink_ == "")
    {
        ShellDisplay::error("[PR2JointsPerception] No head link has been defined");
        return false;
    }

    links_to_entity_ = {{right_hand_link_, owds::BODY_PART_RIGHT_HAND},
                        {left_hand_link_, owds::BODY_PART_LEFT_HAND},
                        {head_ink_, owds::BODY_PART_HEAD}};

    loadPr2Model();

    auto p = bullet_client_->findJointAndLinkIndices(robot_bullet_id_);
    joint_name_id_ = p.first;
    links_name_id_ = p.second;
    for (const auto& link_pair : links_to_entity_)
    {
        percepts_.emplace(link_pair.first, link_pair.first);
        percepts_.at(link_pair.first).setAgentName(robot_name_);
        percepts_.at(link_pair.first).setType(link_pair.second);
        if (links_name_id_.count(link_pair.first) == 0)
        {
            std::cout << "Error: link name '" << link_pair.first
                      << "' passed as 'link_to_entity_names' of ctor of PR2JointsPerception does not exist in Bullet.";
            throw std::runtime_error("Link name '" + link_pair.first + "' not found in Bullet.");
        }
    }
    percepts_.emplace("base_footprint", BodyPart("base_footprint"));
    percepts_.at("base_footprint").setAgentName(robot_name_);
    percepts_.at("base_footprint").setType(BODY_PART_BASE);

    // We set the links mass to 0 to make them static and not be impacted by the gravity
    for(auto& link : links_name_id_)
        bullet_client_->setMass(robot_bullet_id_, link.second, 0);

    return true;
}

bool PR2JointsPerception::perceptionCallback(const sensor_msgs::JointState& msg)
{
    if ((ros::Time::now() - last_update_).toSec() < min_period_)
        return false;
    
    geometry_msgs::TransformStamped robot_base;
    try {
        robot_base = tf_buffer_.lookupTransform("map", "base_footprint", msg.header.stamp, ros::Duration(1.0));
    }
    catch(...) {
        return false;
    }

    bullet_client_->resetBasePositionAndOrientation(
        robot_bullet_id_, {robot_base.transform.translation.x, robot_base.transform.translation.y, robot_base.transform.translation.z},
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
        bullet_client_->resetJointState(robot_bullet_id_, joint_name_id_[name], msg.position[i]);
    }
    for (const auto& link_pair : links_to_entity_)
    {
        b3LinkState link = bullet_client_->getLinkState(robot_bullet_id_, links_name_id_[link_pair.first]);
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
	
	bullet_client_->setAdditionalSearchPath(path_overworld + "/models");

    std::string urdf = n_->param<std::string>("/robot_description", "");
    if (urdf == "")
	    robot_bullet_id_ = bullet_client_->loadURDF("pr2.urdf", {0,0,0}, {0,0,0,1});
    else
        robot_bullet_id_ = bullet_client_->loadURDFRaw(urdf, "pr2_tmp.urdf", {0,0,0}, {0,0,0,1});
}

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::PR2JointsPerception, owds::PerceptionModuleBase_<owds::BodyPart>)