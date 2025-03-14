#ifndef OWDS_STATICOBJECTSPERCEPTIONMODULE_H
#define OWDS_STATICOBJECTSPERCEPTIONMODULE_H

#include "ontologenius/OntologiesManipulator.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

namespace owds {

  class StaticObjectsPerceptionModule : public PerceptionModuleBase_<Object>
  {
  public:
    StaticObjectsPerceptionModule();

    void setParameter(const std::string& parameter_name, const std::string& parameter_value) override;
    bool closeInitialization() override;

  private:
    onto::OntologiesManipulator* ontologies_manipulator_;
    onto::OntologyManipulator* onto_;

    std::string config_file_;

    void addObject(const std::string& name,
                   const std::string& visual_mesh,
                   const std::string& colision_mesh,
                   const std::array<double, 3>& translation,
                   const std::array<double, 3>& rotation,
                   const std::array<double, 4>& color,
                   const std::string& texture = "");

    void addObject(const std::string& name,
                   const std::array<double, 3>& translation,
                   const std::array<double, 3>& rotation,
                   const std::array<double,3>& scale);

    void addVisual(const std::string& name,
                   const std::string& mesh,
                   const std::array<double, 3>& translation,
                   const std::array<double, 3>& rotation,
                   const std::array<double,3>& scale);

    bool readConfiguration(std::string path);
  };

} // namespace owds

#endif // OWDS_STATICOBJECTSPERCEPTIONMODULE_H