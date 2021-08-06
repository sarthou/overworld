#ifndef OWDS_HAND_H
#define OWDS_HAND_H

#include "overworld/BasicTypes/BodyPart.h"

#include <map>
#include <string>

namespace owds {

class Object;

class Hand : public BodyPart
{
public:
  Hand(const std::string& id, bool is_true_id = true);

  void putInHand(Object* object);
  //should be remove from hand through the object function
  void removeFromHand(const std::string& object_name);
  bool isInHand(const std::string& object_name) const ;
  bool isEmpty() const { return (has_in_.size() == 0); }
  std::vector<std::string> getInHand() const;

private:
  std::map<std::string, Object*> has_in_;
};

} //namespace owds

#endif // OWDS_HAND_H