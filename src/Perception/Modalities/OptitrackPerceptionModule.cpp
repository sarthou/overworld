#include "overworld/Perception/Modalities/OptitrackPerceptionModule.h"

namespace owds {
OptitrackPerceptionModule::OptitrackPerceptionModule(ros::NodeHandle* n, const std::string& human_name, const std::array<double,3>& offset)
    : human_name_(human_name), ontologies_manipulator_(n)
{
    mocap_offset_ = Pose(offset, {0,0,0,1});

    ontologies_manipulator_.waitInit();
    ontologies_manipulator_.add("robot");
    onto_ = ontologies_manipulator_.get("robot");
    onto_->close();

    head_name_ = onto_->individuals.getOn(human_name_, "hasHead").at(0);
    left_hand_name_ = onto_->individuals.getOn(human_name_, "hasLeftHand").at(0);
    right_hand_name_ = onto_->individuals.getOn(human_name_, "hasRightHand").at(0);

    auto head_meshs = onto_->individuals.getOn(head_name_, "hasMesh");
    auto left_hand_meshs = onto_->individuals.getOn(left_hand_name_, "hasMesh");
    auto right_hand_meshs = onto_->individuals.getOn(right_hand_name_, "hasMesh");

    percepts_.emplace(head_name_, head_name_);
    percepts_.at(head_name_).setAgentName(human_name_);
    percepts_.at(head_name_).setType(BodyPartType_e::BODY_PART_HEAD);
    Shape_t head_shape, left_hand_shape, right_hand_shape;
    head_shape.color = {0.976470588, 0.894117647, 0.717647059};
    if (head_meshs.size())
    {
        head_shape.type = SHAPE_MESH;
        head_shape.mesh_resource = head_meshs[0].substr(head_meshs[0].find("#") + 1);
    }
    else
    {
        head_shape.type = SHAPE_SPEHERE;
        head_shape.scale = {0.12, 0.15, 0.2};
    }
    percepts_.at(head_name_).setShape(head_shape);

    percepts_.emplace(left_hand_name_, left_hand_name_);
    percepts_.at(left_hand_name_).setAgentName(human_name_);
    percepts_.at(left_hand_name_).setType(BodyPartType_e::BODY_PART_LEFT_HAND);
    left_hand_shape.color = {0.976470588, 0.894117647, 0.717647059};
    if (left_hand_meshs.size())
    {
        left_hand_shape.type = SHAPE_MESH;
        left_hand_shape.mesh_resource = left_hand_meshs[0].substr(left_hand_meshs[0].find("#") + 1);
    }
    else
    {
        left_hand_shape.type = SHAPE_CUBE;
        left_hand_shape.scale = {0.10, 0.03, 0.18};
    }
    percepts_.at(left_hand_name_).setShape(left_hand_shape);

    percepts_.emplace(right_hand_name_, right_hand_name_);
    percepts_.at(right_hand_name_).setAgentName(human_name_);
    percepts_.at(right_hand_name_).setType(BodyPartType_e::BODY_PART_RIGHT_HAND);
    right_hand_shape.color = {0.976470588, 0.894117647, 0.717647059};
    if (right_hand_meshs.size())
    {
        right_hand_shape.type = SHAPE_MESH;
        right_hand_shape.mesh_resource = right_hand_meshs[0].substr(right_hand_meshs[0].find("#") + 1);
    }
    else
    {
        right_hand_shape.type = SHAPE_CUBE;
        right_hand_shape.scale = {0.10, 0.03, 0.18};
    }
    percepts_.at(right_hand_name_).setShape(right_hand_shape);

    optitrack_head_sub_ = n->subscribe("/optitrack/bodies/" + head_name_, 1, &OptitrackPerceptionModule::headRosCallback, this);
    optitrack_left_hand_sub_ = n->subscribe("/optitrack/bodies/" + left_hand_name_, 1, &OptitrackPerceptionModule::leftHandRosCallback, this);
    optitrack_right_hand_sub_ = n->subscribe("/optitrack/bodies/" + right_hand_name_, 1, &OptitrackPerceptionModule::rightHandRosCallback, this);
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

void OptitrackPerceptionModule::headRosCallback(const optitrack_ros::or_pose_estimator_state& msg)
{
    privatePerceptionCallback(std::make_pair(BodyPartType_e::BODY_PART_HEAD, msg));
}

void OptitrackPerceptionModule::leftHandRosCallback(const optitrack_ros::or_pose_estimator_state& msg)
{
    privatePerceptionCallback(std::make_pair(BodyPartType_e::BODY_PART_LEFT_HAND, msg));
}

void OptitrackPerceptionModule::rightHandRosCallback(const optitrack_ros::or_pose_estimator_state& msg)
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
}// namespace owds
