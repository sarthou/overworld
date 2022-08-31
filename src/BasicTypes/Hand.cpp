#include "overworld/BasicTypes/Hand.h"

#include "overworld/BasicTypes/Object.h"

namespace owds {

Hand::Hand(const std::string& id, bool is_true_id) : BodyPart(id, is_true_id)
{}

Hand::Hand(const BodyPart& body_part) : BodyPart(body_part)
{}

void Hand::putInHand(Object* object)
{
  if(has_in_.find(object->id()) == has_in_.end())
  {
    has_in_.insert(std::make_pair(object->id(), object));
    object->setInHand(this);
  }
}

void Hand::removeFromHand(const std::string& object_name)
{
  if(has_in_.erase(object_name) == 0)
  {
    Object* obj_true = nullptr;
    for(auto& obj : has_in_)
    {
      auto false_ids = obj.second->getFalseIds();
      if(false_ids.find(object_name) != false_ids.end())
      {
        obj_true = obj.second;
        break;
      }
    }

    if(obj_true != nullptr)
      obj_true->removeFromHand();
  }
}

bool Hand::isInHand(const std::string& object_name) const
{
  return (has_in_.find(object_name) != has_in_.end());
}

std::vector<std::string> Hand::getInHand() const
{
  std::vector<std::string> res;
  for(auto object : has_in_)
    res.push_back(object.first);
  return res;
}

} // namespace owds