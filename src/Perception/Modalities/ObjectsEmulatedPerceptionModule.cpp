#include "overworld/Perception/Modalities/ObjectsEmulatedPerceptionModule.h"

namespace owds {

bool ObjectsEmulatedPerceptionModule::perceptionCallback(const std::vector<Object*>& msg)
{
  for(auto& percept : percepts_)
    percept.second.setUnseen();

  for(auto object : msg)
  {
    auto percept_it = percepts_.find(object->id());
    if(percept_it == percepts_.end())
      percept_it =  createNewPercept(object);
    percept_it->second.updatePose(object->pose());
  }

  return true;
}

std::map<std::string,Object>::iterator ObjectsEmulatedPerceptionModule::createNewPercept(Object* object)
{
  Object percept(object->id());
  Shape_t shape = object->getShape();
  percept.setShape(shape);

  return percepts_.insert(std::make_pair(percept.id(), percept)).first;
}

} // namespace owds