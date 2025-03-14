#ifndef OWDS_AGENT_H
#define OWDS_AGENT_H

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/FieldOfView.h"
#include "overworld/BasicTypes/Hand.h"

namespace owds {

  enum AgentType_e
  {
    ROBOT = 0,
    HUMAN
  };
  class Agent
  {
  public:
    Agent(const std::string& id, const AgentType_e type) : id_(id),
                                                           type_(type),
                                                           head_(nullptr),
                                                           left_hand_(nullptr),
                                                           right_hand_(nullptr),
                                                           base_(nullptr),
                                                           torso_(nullptr)
    {}

    const std::string& getId() const { return id_; }
    AgentType_e getType() const { return type_; }

    void setHead(BodyPart* head) { head_ = head; }
    void setLeftHand(Hand* hand) { left_hand_ = hand; }
    void setRightHand(Hand* hand) { right_hand_ = hand; }
    void setBase(BodyPart* base) { base_ = base; }
    void setTorso(BodyPart* torso) { torso_ = torso; }
    void setSensor(Sensor* sensor) { agent_sensors_.emplace(sensor->id(), sensor); }

    BodyPart* getHead() const { return head_; }
    Hand* getLeftHand() const { return left_hand_; }
    Hand* getRightHand() const { return right_hand_; }
    BodyPart* getBase() const { return base_; }
    BodyPart* getTorso() const { return torso_; }
    Sensor* getSensor(const std::string& sensor_name)
    {
      auto it = agent_sensors_.find(sensor_name);
      if(it == agent_sensors_.end())
        return nullptr;
      else
        return it->second;
    }
    std::map<std::string, Sensor*> getSensors() { return agent_sensors_; }

  private:
    std::string id_;
    AgentType_e type_;

    BodyPart* head_;
    Hand* left_hand_;
    Hand* right_hand_;
    BodyPart* base_;
    BodyPart* torso_;
    std::map<std::string, Sensor*> agent_sensors_;
  };

} // namespace owds

#endif // OWDS_AGENT_H