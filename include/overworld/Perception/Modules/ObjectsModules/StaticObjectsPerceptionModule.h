#ifndef OWDS_STATICOBJECTSPERCEPTIONMODULE_H
#define OWDS_STATICOBJECTSPERCEPTIONMODULE_H

#include "overworld/BasicTypes/Object.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

namespace owds {

class StaticObjectsPerceptionModule : public PerceptionModuleBase_<Object>
{
public:
    StaticObjectsPerceptionModule();
private:
    void addObject(const std::string& name,
                   const std::string& file,
                   const std::array<double,3>& translation,
                   const std::array<double,3>& rotation,
                   const std::array<double,3>& color);
};

} // namespace owds

#endif // OWDS_STATICOBJECTSPERCEPTIONMODULE_H