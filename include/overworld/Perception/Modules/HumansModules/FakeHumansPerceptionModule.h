#ifndef OWDS_FAKEHUMANPERCEPTIONMODULE_H
#define OWDS_FAKEHUMANPERCEPTIONMODULE_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include "overworld/AgentsPose.h"

#include "ontologenius/OntologiesManipulator.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace owds {


class FakeHumansPerceptionModule : public PerceptionModuleRosBase<BodyPart, overworld::AgentsPose>
{
  public:
    FakeHumansPerceptionModule();

    virtual bool closeInitialization() override;

  private:
    bool perceptionCallback(const overworld::AgentsPose& msg);

    Percept<BodyPart> createPercept(const std::string& human_name, const std::string& part_name);

    onto::OntologiesManipulator* ontologies_manipulator_;
    onto::OntologyManipulator* onto_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_listener_;
};

} // namespace owds

#endif // OWDS_FAKEHUMANPERCEPTIONMODULE_H
