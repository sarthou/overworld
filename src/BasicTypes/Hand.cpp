#include "overworld/BasicTypes/Hand.h"

#include "overworld/BasicTypes/Object.h"

namespace owds {

Hand::Hand(const std::string& id, bool is_true_id) : BodyPart(id, is_true_id)
{}

Hand::Hand(const BodyPart& body_part) : BodyPart(body_part)
{}

void Hand::putInHand(Object* object)
{
  putInHand(object, has_in_);
}

void Hand::putPerceptInHand(Percept<Object>* percept)
{
  putInHand(percept, has_percept_in_);
}

void Hand::removeFromHand(const std::string& object_name)
{
  removeFromHand(object_name, has_in_);
}

void Hand::removePerceptFromHand(const std::string& object_name)
{
  removeFromHand(object_name, has_percept_in_);
}

bool Hand::isInHand(const std::string& object_name) const
{
  return (has_in_.find(object_name) != has_in_.end());
}

bool Hand::isPerceptInHand(const std::string& object_name) const
{
  return (has_percept_in_.find(object_name) != has_percept_in_.end());
}

std::vector<std::string> Hand::getInHand() const
{
  std::vector<std::string> res;
  for(auto object : has_in_)
    res.push_back(object.first);
  return res;
}

std::vector<std::string> Hand::getPerceptsInHand() const
{
  std::vector<std::string> res;
  for(auto percept : has_percept_in_)
    res.push_back(percept.first);
  return res;
}

} // namespace owds