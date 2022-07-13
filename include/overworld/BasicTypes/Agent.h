#ifndef OWDS_AGENT_H
#define OWDS_AGENT_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Hand.h"
#include "overworld/BasicTypes/FieldOfView.h"

namespace owds {

enum AgentType_e
{
  ROBOT = 0,
  HUMAN
};
class Agent
{
public:
  Agent(const std::string& id, const FieldOfView& field_of_view, const AgentType_e type) : id_(id), field_of_view_(field_of_view), type_(type)
  {
    head_ = nullptr;
    left_hand_ = nullptr;
    right_hand_ = nullptr;
    base_ = nullptr;
    torso_ = nullptr;
  }

  const std::string& getId() const { return id_; }
  const FieldOfView& getFieldOfView() const { return field_of_view_; }
  const AgentType_e getType() const { return type_;}
  
  void setHead(BodyPart* head) { head_ = head; }
  void setLeftHand(Hand* hand) { left_hand_ = hand; }
  void setRightHand(Hand* hand) { right_hand_ = hand; }
  void setBase(BodyPart* base) { base_ = base; }
  void setTorso(BodyPart* torso) { torso_ = torso; }

  BodyPart* getHead() const { return head_; }
  Hand* getLeftHand() const { return left_hand_; }
  Hand* getRightHand() const { return right_hand_; }
  BodyPart* getBase() const { return base_; }
  BodyPart* getTorso() const { return torso_; }

private:
  std::string id_;
  FieldOfView field_of_view_;
  AgentType_e type_;

  BodyPart* head_;
  Hand* left_hand_;
  Hand* right_hand_;
  BodyPart* base_;
  BodyPart* torso_;
};

} // namespace owds

#endif // OWDS_AGENT_H