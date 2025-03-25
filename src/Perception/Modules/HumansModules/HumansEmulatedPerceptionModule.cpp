#include "overworld/Perception/Modules/HumansModules/HumansEmulatedPerceptionModule.h"

namespace owds {

  bool HumansEmulatedPerceptionModule::perceptionCallback(const std::vector<BodyPart*>& msg)
  {
    setAllPerceptsUnseen();

    for(auto body_part : msg)
    {
      auto percept_it = percepts_.find(body_part->id());
      if(percept_it == percepts_.end())
        percept_it = createNewPercept(body_part);
      percept_it->second.updatePose(body_part->pose());
    }

    return true;
  }

  std::map<std::string, Percept<BodyPart>>::iterator HumansEmulatedPerceptionModule::createNewPercept(BodyPart* object)
  {
    Percept<BodyPart> percept(object->id());
    Shape_t shape = object->getShape();
    percept.setShape(shape);
    percept.setAgentName(object->getAgentName());
    percept.setType(object->getType());

    return percepts_.insert(std::make_pair(percept.id(), percept)).first;
  }

} // namespace owds