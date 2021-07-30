#ifndef OWDS_AGENT_H
#define OWDS_AGENT_H

#include "overworld/BasicTypes/BodyPart.h"

namespace owds {

class Agent
{
public:
  BodyPart* head;
  BodyPart* left_hand;
  BodyPart* right_hand;
private:
};

} // namespace owds

#endif // OWDS_AGENT_H