#ifndef OWDS_AGENT_H
#define OWDS_AGENT_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/FieldOfView.h"

namespace owds {

class Agent
{
public:
  Agent(const std::string id, const FieldOfView& field_of_view) : id_(id), field_of_view_(field_of_view)
  {
    head_ = nullptr;
    left_hand_ = nullptr;
    right_hand_ = nullptr;
  }

  std::string getId() { return id_; }
  const FieldOfView& getFieldOfView() { return field_of_view_; }
  
  void setHead(BodyPart* head) { head_ = head; }
  void setLeftHand(BodyPart* hand) { left_hand_ = hand; }
  void setRightHand(BodyPart* hand) { right_hand_ = hand; }

  BodyPart* getHead() { return head_; }
  BodyPart* getLeftHand() { return left_hand_; }
  BodyPart* getRightHand() { return right_hand_; }

private:
  std::string id_;
  FieldOfView field_of_view_;

  BodyPart* head_;
  BodyPart* left_hand_;
  BodyPart* right_hand_;
};

} // namespace owds

#endif // OWDS_AGENT_H