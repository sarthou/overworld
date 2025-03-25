#include "overworld/Perception/Modules/AreasModules/AreasEmulatedPerceptionModule.h"

namespace owds {

  bool AreasEmulatedPerceptionModule::perceptionCallback(const std::vector<Area*>& msg)
  {
    for(auto area : msg)
    {
      auto percept_it = percepts_.find(area->id());
      if(percept_it == percepts_.end())
        percept_it = createNewPercept(area);
    }

    return true;
  }

  std::map<std::string, Percept<Area>>::iterator AreasEmulatedPerceptionModule::createNewPercept(Area* area)
  {
    Percept<Area> percept(*area);
    percept.setOwner(nullptr);
    percept.clearInsideEntities();
    percept.setWorldLineIds({});
    percept.setWorldTextIds({});

    updated_ = true;

    return percepts_.insert(std::make_pair(percept.id(), percept)).first;
  }

} // namespace owds