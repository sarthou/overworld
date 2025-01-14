#include "overworld/Perception/Modules/RobotsModules/JointStatePerceptionModule.h"

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

namespace owds {

  JointStatePerceptionModule::JointStatePerceptionModule() : PerceptionModuleRosBase("/joint_states"),
                                                             tf2_listener_(tf_buffer_),
                                                             base_link_("base_footprint"),
                                                             ontologies_manipulator_(nullptr),
                                                             onto_(nullptr)
  {
    min_period_ = 0.005;
  }

  void JointStatePerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
  {
    if(parameter_name == "min_period")
      min_period_ = std::stod(parameter_value);
    else if(parameter_name == "robot_name")
      robot_name_ = parameter_value;
    else
      ShellDisplay::warning("[JointStatePerceptionModule] Unkown parameter " + parameter_name);
  }

  bool JointStatePerceptionModule::closeInitialization()
  {
    if(robot_name_ == "")
    {
      ShellDisplay::error("[JointStatePerceptionModule] No robot name has been defined");
      return false;
    }

    ontologies_manipulator_ = new onto::OntologiesManipulator();
    ontologies_manipulator_->waitInit();
    ontologies_manipulator_->add(robot_name_);
    onto_ = ontologies_manipulator_->get(robot_name_);
    onto_->close();

    auto head_names = onto_->individuals.getOn(robot_name_, "hasHead");
    auto left_hand_names = onto_->individuals.getOn(robot_name_, "hasLeftHand");
    auto right_hand_names = onto_->individuals.getOn(robot_name_, "hasRightHand");
    auto base_names = onto_->individuals.getOn(robot_name_, "hasBase");

    if(head_names.size())
      head_link_ = head_names.front();
    if(left_hand_names.size())
      left_hand_link_ = left_hand_names.front();
    if(right_hand_names.size())
      right_hand_link_ = right_hand_names.front();
    if(base_names.size())
      base_link_ = base_names.front();

    if(head_link_ == "")
    {
      ShellDisplay::error("[JointStatePerceptionModule] No head link has been defined");
      return false;
    }

    links_to_entity_ = {
      {head_link_, owds::BODY_PART_HEAD}
    };
    if(right_hand_link_ != "")
      links_to_entity_.emplace_back(right_hand_link_, owds::BODY_PART_RIGHT_HAND);
    if(left_hand_link_ != "")
      links_to_entity_.emplace_back(left_hand_link_, owds::BODY_PART_LEFT_HAND);

    auto robot_body_parts = onto_->individuals.getOn(robot_name_, "hasBodyPart");
    for(const auto& part : robot_body_parts)
    {
      auto robot_sensors_ids = onto_->individuals.getOn(part, "hasSensor");
      for(const auto& robot_sensor_id : robot_sensors_ids)
      {
        auto vect = onto_->individuals.getOn(robot_sensor_id, "hasFrameId");
        if(!vect.empty())
        {
          links_to_entity_.emplace_back(vect.front(), owds::BODY_PART_SENSOR);
        }
        else
          links_to_entity_.emplace_back(robot_sensor_id, owds::BODY_PART_SENSOR);
      }
    }
    loadRobotModel();

    auto p = bullet_client_->findJointAndLinkIndices(robot_bullet_id_);
    joint_name_id_ = p.first;
    links_name_id_ = p.second;
    for(const auto& link_pair : links_to_entity_)
    {
      percepts_.emplace(link_pair.first, link_pair.first);
      percepts_.at(link_pair.first).setAgentName(robot_name_);
      percepts_.at(link_pair.first).setType(link_pair.second);
      // we set the bullet id of the parent to inform the manager
      percepts_.at(link_pair.first).setBulletId(robot_bullet_id_);
      if(links_name_id_.count(link_pair.first) == 0)
      {
        std::cout << "Error: link name '" << link_pair.first
                  << "' passed as 'link_to_entity_names' of ctor of JointStatePerceptionModule does not exist in Bullet.";
        throw std::runtime_error("Link name '" + link_pair.first + "' not found in Bullet.");
      }
    }
    percepts_.emplace(base_link_, BodyPart(base_link_));
    percepts_.at(base_link_).setAgentName(robot_name_);
    percepts_.at(base_link_).setType(BODY_PART_BASE);
    // we set the bullet id of the parent to inform the manager
    percepts_.at(base_link_).setBulletId(robot_bullet_id_);

    if(updateBasePose() == false)
      ShellDisplay::warning("[JointStatePerceptionModule] Pr2 base has no position in tf");
    else
      updated_ = true;

    // We set the links mass to 0 to make them static and not be impacted by the gravity
    for(auto& link : links_name_id_)
      bullet_client_->setMass(robot_bullet_id_, link.second, 0);

    return true;
  }

  bool JointStatePerceptionModule::perceptionCallback(const sensor_msgs::JointState& msg)
  {
    if((ros::Time::now() - last_update_).toSec() < min_period_)
      return false;

    if(updateBasePose(msg.header.stamp) == false)
      return false;

    for(size_t i = 0; i < msg.name.size(); i++)
    {
      std::string name = msg.name[i];
      if(joint_name_id_.count(name) != 1)
      {
        // std::cout << "Joint name not found in Bullet: " << name << std::endl;
        continue;
      }
      bullet_client_->resetJointState(robot_bullet_id_, joint_name_id_[name], msg.position[i]);
    }
    for(const auto& link_pair : links_to_entity_)
    {
      b3LinkState link = bullet_client_->getLinkState(robot_bullet_id_, links_name_id_.at(link_pair.first));
      double* pos = link.m_worldLinkFramePosition;
      double* rot = link.m_worldLinkFrameOrientation;
      percepts_.at(link_pair.first).updatePose({
                                                 {pos[0], pos[1], pos[2]}
      },
                                               {{rot[0], rot[1], rot[2], rot[3]}}, msg.header.stamp);
    }
    last_update_ = ros::Time::now();
    return true;
  }

  bool JointStatePerceptionModule::updateBasePose(const ros::Time& stamp)
  {
    geometry_msgs::TransformStamped robot_base;
    try
    {
      robot_base = tf_buffer_.lookupTransform("map", base_link_, stamp, ros::Duration(1.0));
    }
    catch(const tf2::TransformException& ex)
    {
      ShellDisplay::error("[JointStatePerceptionModule]" + std::string(ex.what()));
      return false;
    }
    catch(...)
    {
      return false;
    }

    bullet_client_->resetBasePositionAndOrientation(
      robot_bullet_id_, {robot_base.transform.translation.x, robot_base.transform.translation.y, robot_base.transform.translation.z},
      {robot_base.transform.rotation.x, robot_base.transform.rotation.y, robot_base.transform.rotation.z, robot_base.transform.rotation.w});
    percepts_.at(base_link_)
      .updatePose(
        {robot_base.transform.translation.x, robot_base.transform.translation.y, robot_base.transform.translation.z},
        {robot_base.transform.rotation.x, robot_base.transform.rotation.y, robot_base.transform.rotation.z, robot_base.transform.rotation.w});

    return true;
  }

  void JointStatePerceptionModule::loadRobotModel()
  {
    std::string path_overworld = ros::package::getPath("overworld");

    bullet_client_->setAdditionalSearchPath(path_overworld + "/models");

    std::string urdf = n_->param<std::string>("/robot_description", "");
    if(urdf == "")
      robot_bullet_id_ = bullet_client_->loadURDF(robot_name_ + ".urdf", {0, 0, 0}, {0, 0, 0, 1}, true, URDF_USE_MATERIAL_COLORS_FROM_MTL);
    else
      robot_bullet_id_ = bullet_client_->loadURDFRaw(urdf, robot_name_ + "_tmp.urdf", {0, 0, 0}, {0, 0, 0, 1}, true, URDF_USE_MATERIAL_COLORS_FROM_MTL);
  }

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::JointStatePerceptionModule, owds::PerceptionModuleBase_<owds::BodyPart>)