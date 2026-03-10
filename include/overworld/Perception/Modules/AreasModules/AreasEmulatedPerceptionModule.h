#ifndef OWDS_AREASEMULATEDPERCEPTIONMODULE_H
#define OWDS_AREASEMULATEDPERCEPTIONMODULE_H

#include <vector>

#include "overworld/Perception/Modules/PerceptionModuleBase.h"
// PerceptionModuleBase should be included first
#include "overworld/BasicTypes/Area.h"

namespace owds {

  class AreasEmulatedPerceptionModule : public PerceptionModuleBase<Area, std::vector<Area*>>
  {
  public:
    AreasEmulatedPerceptionModule() = default;

  private:
    bool perceptionCallback(const std::vector<Area*>& msg) override;

    std::map<std::string, Percept<Area>>::iterator createNewPercept(Area* area);
  };

} // namespace owds

#endif // OWDS_AREASEMULATEDPERCEPTIONMODULE_H