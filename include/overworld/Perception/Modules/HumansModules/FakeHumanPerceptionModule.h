#ifndef OWDS_FAKEHUMANPERCEPTIONMODULE_H
#define OWDS_FAKEHUMANPERCEPTIONMODULE_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include "overworld/AgentPose.h"

#include "ontologenius/OntologiesManipulator.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace owds {


class FakeHumanPerceptionModule : public PerceptionModuleRosBase<BodyPart, overworld::AgentPose>
{
  public:
    FakeHumanPerceptionModule();

    virtual bool closeInitialization() override;

  private:
    bool perceptionCallback(const overworld::AgentPose& msg);

    BodyPart createBodyPart(const std::string& human_name, const std::string& part_name);

    OntologiesManipulator* ontologies_manipulator_;
    OntologyManipulator* onto_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_listener_;
};

} // namespace owds

#endif // OWDS_FAKEHUMANPERCEPTIONMODULE_H
