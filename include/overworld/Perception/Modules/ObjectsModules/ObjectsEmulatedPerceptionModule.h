#ifndef OWDS_OBJECTSEMULATEDPERCEPTIONMODULE_H
#define OWDS_OBJECTSEMULATEDPERCEPTIONMODULE_H

#include <vector>
#include <map>

#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/Percept.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

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