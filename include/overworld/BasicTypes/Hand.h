#ifndef OWDS_HAND_H
#define OWDS_HAND_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Percept.h"

#include <map>
#include <string>

namespace owds {

class Object;

class Hand : public BodyPart
{
public:
  explicit Hand(const std::string& id, bool is_true_id = true);
  explicit Hand(const BodyPart& body_part);

  void putInHand(Object* object);
  void putPerceptInHand(Percept<Object>* percept);

  //should be remove from hand through the object function
  void removeFromHand(const std::string& object_name);
  void removePerceptFromHand(const std::string& percept_name);

  bool isInHand(const std::string& object_name) const;
  bool isPerceptInHand(const std::string& object_name) const;

  bool isEmpty() const { return (has_in_.size() == 0); }
  bool isPerceptEmpty() const { return (has_percept_in_.size() == 0); }

  std::vector<std::string> getInHand() const;
  std::vector<std::string> getPerceptsInHand() const;

private:
  std::map<std::string, Object*> has_in_;
  std::map<std::string, Percept<Object>*> has_percept_in_;

  template<typename T>
  void putInHand(T* object, std::map<std::string, T*>& map_has_in)
  {
    if(map_has_in.find(object->id()) == map_has_in.end())
    {
      map_has_in.insert(std::make_pair(object->id(), object));
      object->setInHand(this);
    }
  }

  template<typename T>
  void removeFromHand(const std::string& object_name, std::map<std::string, T*>& map_has_in)
  {
    if(map_has_in.erase(object_name) == 0)
    {
      T* obj_true = nullptr;
      for(auto& obj : map_has_in)
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
};

} //namespace owds

#endif // OWDS_HAND_H