#ifndef OWDS_HUMANSEMULATEDPERCEPTIONMODULE_H
#define OWDS_HUMANSEMULATEDPERCEPTIONMODULE_H

#include <vector>

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

namespace owds {

  class HumansEmulatedPerceptionModule : public PerceptionModuleBase<BodyPart, std::vector<BodyPart*>>
  {
  public:
    HumansEmulatedPerceptionModule() = default;

  private:
    bool perceptionCallback(const std::vector<BodyPart*>& msg) override;

    std::map<std::string, Percept<BodyPart>>::iterator createNewPercept(BodyPart* object);
  };

} // namespace owds

#endif // OWDS_HUMANSEMULATEDPERCEPTIONMODULE_H