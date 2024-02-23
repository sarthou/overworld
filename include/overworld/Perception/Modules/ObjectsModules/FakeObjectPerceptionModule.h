#ifndef OWDS_FAKEOBJECTPERCEPTIONMODULE_H
#define OWDS_FAKEOBJECTPERCEPTIONMODULE_H

#include "overworld/BasicTypes/Object.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include "overworld/EntitiesPoses.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace owds {

class FakeObjectPerceptionModule : public PerceptionModuleRosBase<Object, overworld::EntitiesPoses>
{
public:
  FakeObjectPerceptionModule();

  bool closeInitialization() override;

private:
  onto::OntologiesManipulator* ontologies_manipulator_;
  onto::OntologyManipulator* onto_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  virtual bool perceptionCallback(const overworld::EntitiesPoses& msg) override;

  Object createNewEntity(const std::string& id);
};

} // namespace owds

#endif // OWDS_FAKEOBJECTPERCEPTIONMODULE_H