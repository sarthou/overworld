#ifndef OWDS_FAKEOBJECTPERCEPTIONMODULE_H
#define OWDS_FAKEOBJECTPERCEPTIONMODULE_H

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "overworld/BasicTypes/Object.h"
#include "overworld/EntitiesPoses.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

namespace owds {

  class FakeObjectPerceptionModule : public PerceptionModuleRosBase<Object, overworld::EntitiesPoses>
  {
  public:
    FakeObjectPerceptionModule();

    bool closeInitialization() override;
    void setParameter(const std::string& parameter_name, const std::string& parameter_value) override;

  private:
    onto::OntologiesManipulator* ontologies_manipulator_;
    onto::OntologyManipulator* onto_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    std::string topic_name_;
    bool true_id_;
    std::string sensor_id_;

    virtual bool perceptionCallback(const overworld::EntitiesPoses& msg) override;

    Percept<Object> createPercept(const std::string& id);
  };

} // namespace owds

#endif // OWDS_FAKEOBJECTPERCEPTIONMODULE_H