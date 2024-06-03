#ifndef OWDS_OBJAREASPERCEPTIONMODULE_H
#define OWDS_OBJAREASPERCEPTIONMODULE_H

#include "overworld/BasicTypes/Area.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

namespace owds {

  class ObjAreasPerceptionModule : public PerceptionModuleBase_<Area>
  {
  public:
    ObjAreasPerceptionModule();

    virtual void setParameter(const std::string& parameter_name, const std::string& parameter_value) override;
    virtual bool closeInitialization() override;

  private:
    std::string config_file_;

    void addCircle(const std::string& id, const std::array<double, 3>& pose, double radius, double half_height, double hysteresis, const std::string& owner);
    void addPolygon(const std::string& id, const std::string& polygon_path, const std::array<double, 3>& pose, double z_min, double z_max, double hysteresis, const std::string& owner);
  };

} // namespace owds

#endif // OWDS_OBJAREASPERCEPTIONMODULE_H