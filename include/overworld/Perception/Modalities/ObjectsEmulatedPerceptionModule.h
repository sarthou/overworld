#ifndef OWDS_OBJECTSEMULATEDPERCEPTIONMODULE_H
#define OWDS_OBJECTSEMULATEDPERCEPTIONMODULE_H

#include "overworld/BasicTypes/Object.h"
#include "overworld/Perception/PerceptionModuleBase.h"

#include <vector>

namespace owds {

class ObjectsEmulatedPerceptionModule : public PerceptionModuleBase<Object, std::vector<Object*>>
{
public:
  ObjectsEmulatedPerceptionModule() = default;
private:
  bool perceptionCallback(const std::vector<Object*>& msg);

  std::map<std::string,Object>::iterator createNewPercept(Object* object);
};

} // namespace owds

#endif // OWDS_OBJECTSEMULATEDPERCEPTIONMODULE_H