#ifndef OWDS_FAKEROBOTPERCEPTIONMODULE_H
#define OWDS_FAKEROBOTPERCEPTIONMODULE_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include "overworld/AgentPose.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace owds {

class FakeRobotPerceptionModule : public owds::PerceptionModuleRosBase<owds::BodyPart, overworld::AgentPose>
{
  public:
    FakeRobotPerceptionModule();
    virtual ~FakeRobotPerceptionModule() = default;

    virtual void setParameter(const std::string& parameter_name, const std::string& parameter_value) override;
    virtual bool closeInitialization() override;

    virtual std::string getAgentName() override { return robot_name_; } 

  protected:
    bool perceptionCallback(const overworld::AgentPose& msg) override;
    std::string robot_name_;

    std::string head_link_;
    std::string base_link_;

    Pose head_pose_;
    Pose base_pose_;

    OntologiesManipulator* ontologies_manipulator_;
    OntologyManipulator* onto_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    BodyPart createBodyPart(const std::string& robot_name, const std::string& part_name, BodyPartType_e part_type);
    Pose stringToPose(const std::string& str);
};

} // namespace owds

#endif /* OWDS_FAKEROBOTPERCEPTIONMODULE_H */
