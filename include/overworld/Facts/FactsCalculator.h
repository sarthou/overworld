#ifndef OWDS_FACTSCALCULATOR_H
#define OWDS_FACTSCALCULATOR_H

#include <map>
#include <string>
#include <unordered_set>

#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/Area.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/Facts/Fact.h"
#include "overworld/Engine/Common/WorldTypes.h"

namespace owds {

  class FactsCalculator
  {
  public:
    FactsCalculator() = default;

    std::vector<Fact> computeObjectsFacts(const std::map<std::string, Object*>& objects,
                                          bool clear = true);
    std::vector<Fact> computeAgentsFacts(const std::map<std::string, Object*>& objects,
                                         const std::map<std::string, Agent*>& agents,
                                         const std::map<std::string, std::unordered_set<uint32_t>>& segmantation_ids,
                                         bool clear = true);
    std::vector<Fact> computeAreasFacts(const std::map<std::string, Area*>& areas,
                                        const std::map<std::string, Object*>& objects,
                                        const std::map<std::string, BodyPart*>& body_parts,
                                        bool clear = true);

  private:
    std::vector<Fact> facts_;

    bool isOnTopfOf(Object* object_under, Object* object_on);
    bool isInContainer(Object* object_around, Object* object_in);
    bool overlapXY(const struct AABB_t& aabb_1, const struct AABB_t& aabb_2);

    bool isPerceiving(Agent* agent_perceiving, Agent* agent_perceived);

    bool isLookingAt(Agent* agent, const std::unordered_set<uint32_t>& seen_engine_ids, const Object* object);
    bool hasInHand(Agent* agent, Object* object);
    bool isHandMovingTowards(Agent* agent, Object* object);

    bool isValid(Object* object);
  };

} // namespace owds

#endif // OWDS_FACTSCALCULATOR_H