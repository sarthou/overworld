#ifndef OWDS_HUMANSEMULATEDPERCEPTIONMODULE_H
#define OWDS_HUMANSEMULATEDPERCEPTIONMODULE_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/Perception/PerceptionModuleBase.h"

#include <vector>

namespace owds {

class HumansEmulatedPerceptionModule : public PerceptionModuleBase<BodyPart, std::vector<BodyPart*>>
{
public:
  HumansEmulatedPerceptionModule() = default;

private:
  bool perceptionCallback(const std::vector<BodyPart*>& msg);

  std::map<std::string,BodyPart>::iterator createNewPercept(BodyPart* object);
};

} // namespace owds

#endif // OWDS_HUMANSEMULATEDPERCEPTIONMODULE_H