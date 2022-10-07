#ifndef OWDS_OPTITRACKPERCEPTIONMODULE_H
#define OWDS_OPTITRACKPERCEPTIONMODULE_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include <optitrack_msgs/or_pose_estimator_state.h>

#include "ontologenius/OntologiesManipulator.h"

namespace owds {

typedef std::pair<BodyPartType_e, optitrack_msgs::or_pose_estimator_state> BodyPartOptitrackPose;

class OptitrackPerceptionModule : public PerceptionModuleBase_<BodyPart>
{
  public:
    OptitrackPerceptionModule();

    virtual void setParameter(const std::string& parameter_name, const std::string& parameter_value) override;
    virtual bool closeInitialization() override;

  private:
    void headRosCallback(const optitrack_msgs::or_pose_estimator_state& msg);
    void leftHandRosCallback(const optitrack_msgs::or_pose_estimator_state& msg);
    void rightHandRosCallback(const optitrack_msgs::or_pose_estimator_state& msg);

    void privatePerceptionCallback(const BodyPartOptitrackPose& msg);

  protected:
    bool perceptionCallback(const BodyPartOptitrackPose& msg);

    OntologiesManipulator* ontologies_manipulator_;
    OntologyManipulator* onto_;
    double offset_x_;
    double offset_y_;
    double offset_z_;
    Pose mocap_offset_;

    std::string human_name_;
    std::string head_name_, left_hand_name_, right_hand_name_;

    ros::Subscriber optitrack_head_sub_;
    ros::Subscriber optitrack_left_hand_sub_;
    ros::Subscriber optitrack_right_hand_sub_;
};

} // namespace owds

#endif /* OWDS_OPTITRACKPERCEPTIONMODULE_H */
