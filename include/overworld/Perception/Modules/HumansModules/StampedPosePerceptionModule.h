#ifndef OWDS_STAMPEDPOSEPERCEPTIONMODULE_H
#define OWDS_STAMPEDPOSEPERCEPTIONMODULE_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include <geometry_msgs/PoseStamped.h>

#include "ontologenius/OntologiesManipulator.h"

namespace owds {


class StampedPosePerceptionModule : public PerceptionModuleRosBase<BodyPart, geometry_msgs::PoseStamped>
{
  public:
    StampedPosePerceptionModule();

    virtual void setParameter(const std::string& parameter_name, const std::string& parameter_value) override;
    virtual bool closeInitialization() override;

  protected:
    bool perceptionCallback(const geometry_msgs::PoseStamped& msg) override;

    onto::OntologiesManipulator* ontologies_manipulator_;
    onto::OntologyManipulator* onto_;

    std::string human_name_;
    std::string head_name_;
};

} // namespace owds

#endif /* OWDS_STAMPEDPOSEPERCEPTIONMODULE_H */
