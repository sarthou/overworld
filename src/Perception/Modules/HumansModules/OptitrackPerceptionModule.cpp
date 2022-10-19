#include "overworld/Perception/Modules/HumansModules/OptitrackPerceptionModule.h"

#include "overworld/Utility/ShellDisplay.h"

#include <pluginlib/class_list_macros.h>

namespace owds {

OptitrackPerceptionModule::OptitrackPerceptionModule() : ontologies_manipulator_(nullptr),
                                                         onto_(nullptr)
{
    offset_x_ = 0;
    offset_y_ = 0;
    offset_z_ = 0;
}

void OptitrackPerceptionModule::setParameter(const std::string& parameter_name, const std::string& parameter_value)
{
    if(parameter_name == "name")
        human_name_ = parameter_value;
    else if(parameter_name == "offset_x")
        offset_x_ = std::stod(parameter_value);
    else if(parameter_name == "offset_y")
        offset_y_ = std::stod(parameter_value);
    else if(parameter_name == "offset_z")
        offset_z_ = std::stod(parameter_value);
    else
        ShellDisplay::warning("[OptitrackPerceptionModule] Unkown parameter " + parameter_name);
}

bool OptitrackPerceptionModule::closeInitialization()
{
    if(human_name_ == "")
    {
        ShellDisplay::error("[OptitrackPerceptionModule] No human name has been defined");
        return false;
    }

    mocap_offset_ = Pose({offset_x_, offset_y_, offset_z_}, {0,0,0,1});

    ontologies_manipulator_ = new OntologiesManipulator(n_);
    ontologies_manipulator_->waitInit();
    ontologies_manipulator_->add(robot_agent_->getId());
    onto_ = ontologies_manipulator_->get(robot_agent_->getId());
    onto_->close();

    auto head_names = onto_->individuals.getOn(human_name_, "hasHead");
    auto left_hand_names = onto_->individuals.getOn(human_name_, "hasLeftHand");
    auto right_hand_names = onto_->individuals.getOn(human_name_, "hasRightHand");

    if (head_names.size() == 0 || left_hand_names.size() == 0 || right_hand_names.size() == 0)
    {
        ShellDisplay::error("No body part defined in the ontology for human: '" + human_name_ + "'.");
        throw std::runtime_error("No body part defined in the ontology for human: '" + human_name_ + "'.");
    }

    head_name_ = head_names.at(0);
    left_hand_name_ = left_hand_names.at(0);
    right_hand_name_ = right_hand_names.at(0);

    std::array<double, 3> default_color = {0.976470588, 0.894117647, 0.717647059};

    percepts_.emplace(head_name_, head_name_);
    percepts_.at(head_name_).setAgentName(human_name_);
    percepts_.at(head_name_).setType(BodyPartType_e::BODY_PART_HEAD);
    Shape_t head_shape = PerceptionModuleBase_::getEntityShapeFromOntology(onto_, head_name_);
    if(head_shape.type == SHAPE_NONE)
    {
        head_shape.type = SHAPE_SPEHERE;
        head_shape.scale = {0.12, 0.15, 0.2};
        head_shape.color = PerceptionModuleBase_::getEntityColorFromOntology(onto_, head_name_, default_color);
    }
    percepts_.at(head_name_).setShape(head_shape);

    percepts_.emplace(left_hand_name_, left_hand_name_);
    percepts_.at(left_hand_name_).setAgentName(human_name_);
    percepts_.at(left_hand_name_).setType(BodyPartType_e::BODY_PART_LEFT_HAND);
    Shape_t left_hand_shape = PerceptionModuleBase_::getEntityShapeFromOntology(onto_, left_hand_name_);
    if(left_hand_shape.type == SHAPE_NONE)
    {
        left_hand_shape.type = SHAPE_CUBE;
        left_hand_shape.scale = {0.10, 0.03, 0.18};
        left_hand_shape.color = PerceptionModuleBase_::getEntityColorFromOntology(onto_, left_hand_name_, default_color);
    }
    percepts_.at(left_hand_name_).setShape(left_hand_shape);

    percepts_.emplace(right_hand_name_, right_hand_name_);
    percepts_.at(right_hand_name_).setAgentName(human_name_);
    percepts_.at(right_hand_name_).setType(BodyPartType_e::BODY_PART_RIGHT_HAND);
    Shape_t right_hand_shape = PerceptionModuleBase_::getEntityShapeFromOntology(onto_, right_hand_name_);
    if(right_hand_shape.type == SHAPE_NONE)
    {
        right_hand_shape.type = SHAPE_CUBE;
        right_hand_shape.scale = {0.10, 0.03, 0.18};
        right_hand_shape.color = PerceptionModuleBase_::getEntityColorFromOntology(onto_, right_hand_name_, default_color);
    }
    percepts_.at(right_hand_name_).setShape(right_hand_shape);

    optitrack_head_sub_ = n_->subscribe("/optitrack/bodies/" + head_name_, 1, &OptitrackPerceptionModule::headRosCallback, this);
    optitrack_left_hand_sub_ = n_->subscribe("/optitrack/bodies/" + left_hand_name_, 1, &OptitrackPerceptionModule::leftHandRosCallback, this);
    optitrack_right_hand_sub_ = n_->subscribe("/optitrack/bodies/" + right_hand_name_, 1, &OptitrackPerceptionModule::rightHandRosCallback, this);

    return true;
}

bool OptitrackPerceptionModule::perceptionCallback(const BodyPartOptitrackPose& msg)
{
    if (msg.second.pos.size() == 0)
    {
        return false;
    }
    auto optipos = msg.second.pos[0];
    auto optiatt = msg.second.att[0];
    Pose p({optipos.x, optipos.y, optipos.z}, {optiatt.qx, optiatt.qy, optiatt.qz, optiatt.qw});
    p = mocap_offset_ * p;
    ros::Time stamp(msg.second.ts.sec, msg.second.ts.nsec);
    switch (msg.first)
    {
    case BodyPartType_e::BODY_PART_HEAD:
        percepts_.at(head_name_).updatePose(p, stamp);
        return true;
        break;
    case BodyPartType_e::BODY_PART_LEFT_HAND:
        percepts_.at(left_hand_name_).updatePose(p, stamp);
        return true;
        break;
    case BodyPartType_e::BODY_PART_RIGHT_HAND:
        percepts_.at(right_hand_name_).updatePose(p, stamp);
        return true;
        break;
    default:
        return false;
        break;
    }
    return false;
}

void OptitrackPerceptionModule::headRosCallback(const optitrack_msgs::or_pose_estimator_state& msg)
{
    privatePerceptionCallback(std::make_pair(BodyPartType_e::BODY_PART_HEAD, msg));
}

void OptitrackPerceptionModule::leftHandRosCallback(const optitrack_msgs::or_pose_estimator_state& msg)
{
    privatePerceptionCallback(std::make_pair(BodyPartType_e::BODY_PART_LEFT_HAND, msg));
}

void OptitrackPerceptionModule::rightHandRosCallback(const optitrack_msgs::or_pose_estimator_state& msg)
{
    privatePerceptionCallback(std::make_pair(BodyPartType_e::BODY_PART_RIGHT_HAND, msg));
}

void OptitrackPerceptionModule::privatePerceptionCallback(const BodyPartOptitrackPose& msg){
    if(!this->is_activated_)
      return;

    this->mutex_perception_.lock();
    this->mutex_access_.lock();
    this->mutex_perception_.unlock();
    if(perceptionCallback(msg))
      this->updated_ = true;
    this->mutex_access_.unlock();
}

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::OptitrackPerceptionModule, owds::PerceptionModuleBase_<owds::BodyPart>)
