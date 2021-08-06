#ifndef OWDS_AGENT_H
#define OWDS_AGENT_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Hand.h"
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
    base_ = nullptr;
    torso_ = nullptr;
  }

  std::string getId() { return id_; }
  const FieldOfView& getFieldOfView() { return field_of_view_; }
  
  void setHead(BodyPart* head) { head_ = head; }
  void setLeftHand(Hand* hand) { left_hand_ = hand; }
  void setRightHand(Hand* hand) { right_hand_ = hand; }
  void setBase(BodyPart* base) { base_ = base; }
  void setTorso(BodyPart* torso) { torso_ = torso; }

  BodyPart* getHead() { return head_; }
  Hand* getLeftHand() { return left_hand_; }
  Hand* getRightHand() { return right_hand_; }
  BodyPart* getBase() { return base_; }
  BodyPart* getTorso() { return torso_; }

private:
  std::string id_;
  FieldOfView field_of_view_;

  BodyPart* head_;
  Hand* left_hand_;
  Hand* right_hand_;
  BodyPart* base_;
  BodyPart* torso_;
};

} // namespace owds

#endif // OWDS_AGENT_H