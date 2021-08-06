#ifndef OWDS_FACTSCALCULATOR_H
#define OWDS_FACTSCALCULATOR_H

#include <map>
#include <string>

#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/Agent.h"
#include "overworld/Facts/Fact.h"

namespace owds {

class FactsCalculator
{
public:
  std::vector<Fact> computeFacts(const std::map<std::string, Object*>& objects,
                                 const std::map<std::string, Agent*>& agents);

private:
  std::vector<Fact> facts_;

  bool isOnTopfOf(Object* object_under, Object* object_on);
  bool isInContainer(Object* object_around, Object* object_in);

  bool isInHand(Agent* agent);
  bool isPerceiving(Agent* agent_perceiving, Agent* agent_perceived);

  bool isLookingAt(Agent* agent/*, segmentation_image*/);
};

} // namespace owds

#endif // OWDS_FACTSCALCULATOR_H