#ifndef OWDS_OBJECTSEMULATEDPERCEPTIONMODULE_H
#define OWDS_OBJECTSEMULATEDPERCEPTIONMODULE_H

#include <map>
#include <vector>

#include "overworld/Perception/Modules/PerceptionModuleBase.h"
// PerceptionModuleBase should be included first
#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/Percept.h"

namespace owds {

  class ObjectsEmulatedPerceptionModule : public PerceptionModuleBase<Object, std::vector<Object*>>
  {
  public:
    ObjectsEmulatedPerceptionModule() = default;

  private:
    bool perceptionCallback(const std::vector<Object*>& msg) override;

    std::map<std::string, Percept<Object>>::iterator createNewPercept(Object* object);
  };

} // namespace owds

#endif // OWDS_OBJECTSEMULATEDPERCEPTIONMODULE_H