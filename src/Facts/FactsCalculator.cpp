#include "overworld/Facts/FactsCalculator.h"

namespace owds {

std::vector<Fact> FactsCalculator::computeFacts(const std::map<std::string, Object*>& objects,
                                                const std::map<std::string, Agent*>& agents)
{
  facts_.clear();

  for(auto& obj_from : objects)
  {
    if(obj_from.second->isInHand())
      continue;

    for(auto& obj_to : objects)
    {
      if(obj_to.second->isInHand())
        continue;

      if(obj_from.first != obj_to.first)
      {
        bool is_in = isInContainer(obj_to.second, obj_from.second);
        if(is_in == false)
          isOnTopfOf(obj_to.second, obj_from.second);
      }
    }
  }

  for(auto& agent_from : agents)
  {
    isInHand(agent_from.second);
    for (auto& obj: objects)
    {
      std::unordered_set<int> seen_bullet_ids({});
      isLookingAt(agent_from.second, seen_bullet_ids, obj.second);
    }
    for(auto& agent_to : agents)
    {
      if(agent_from.first != agent_to.first)
      {
        isPerceiving(agent_from.second, agent_to.second);
      }
    }
  }

  return facts_;
}

bool FactsCalculator::isOnTopfOf(Object* object_under, Object* object_on)
{
  if((object_under->isAabbValid() == false) || (object_on->isAabbValid() == false))
    return false;
  else if(overlapXY(object_under->getAabb(), object_on->getAabb()) == false)
    return false;
  else if(std::abs(object_under->getAabb().max[2] - object_on->getAabb().min[2]) > 0.04)
    return false;
  else
  {
    facts_.emplace_back(object_on->id(), "isOnTopOf", object_under->id());
    return true;
  }
}

bool FactsCalculator::isInContainer(Object* object_around, Object* object_in)
{
  if((object_around->isAabbValid() == false) || (object_in->isAabbValid() == false))
    return false;
  else if(object_around->getAabb().min[2] - 0.04 > object_in->getAabb().min[2])
    return false;
  else if(object_around->getAabb().max[2] + 0.04 < object_in->getAabb().max[2])
    return false;
  else if((object_around->getAabb().min[0] > object_in->getAabb().min[0]) || 
          (object_around->getAabb().min[1] > object_in->getAabb().min[1]))
    return false;
  else if((object_around->getAabb().max[0] < object_in->getAabb().max[0]) || 
          (object_around->getAabb().max[1] < object_in->getAabb().max[1]))
    return false;
  else
  {
    facts_.emplace_back(object_in->id(), "isInContainer", object_around->id());
    return true;
  }
}

bool FactsCalculator::overlapXY(const struct aabb_t& aabb_1, const struct aabb_t& aabb_2)
{
  if((aabb_1.min[0] == aabb_1.max[0]) || (aabb_1.min[1] == aabb_1.max[1]) ||
     (aabb_2.min[0] == aabb_2.max[0]) || (aabb_2.min[1] == aabb_2.max[1]))
     return false;
  else if((aabb_1.min[0] >= aabb_2.max[0]) || (aabb_2.min[0] >= aabb_1.max[0]))
    return false;
  else if((aabb_1.min[1] >= aabb_2.max[1]) || (aabb_2.min[1] >= aabb_1.max[1]))
    return false;
  else
    return true;
}

bool FactsCalculator::isInHand(Agent* agent)
{
  bool res = false;
  if(agent->getLeftHand()->isEmpty() == false)
  {
    res = true;
    for(auto& object_name : agent->getLeftHand()->getInHand())
      facts_.emplace_back(agent->getId(), "hasInLeftHand", object_name);
  }

  if(agent->getRightHand()->isEmpty() == false)
  {
    res = true;
    for(auto& object_name : agent->getRightHand()->getInHand())
      facts_.emplace_back(agent->getId(), "hasInRightHand", object_name);
  }

  return res;
}

bool FactsCalculator::isPerceiving(Agent* agent_perceiving, Agent* agent_perceived)
{
  if (agent_perceiving->getHead() == nullptr || agent_perceiving->getHead()->isLocated() == false){
    return false;
  }
  Pose head_pose = agent_perceiving->getHead()->pose();
  if (agent_perceived->getHead() != nullptr && agent_perceived->getHead()->isLocated()){
    if (agent_perceived->getFieldOfView().hasIn(agent_perceived->getHead()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());
      return true;
    }
  }

  if (agent_perceived->getLeftHand() != nullptr && agent_perceived->getLeftHand()->isLocated()){
    if (agent_perceived->getFieldOfView().hasIn(agent_perceived->getLeftHand()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());
      return true;
    }
  }

  if (agent_perceived->getRightHand() != nullptr && agent_perceived->getRightHand()->isLocated()){
    if (agent_perceived->getFieldOfView().hasIn(agent_perceived->getRightHand()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());
      return true;
    }
  }

  if (agent_perceived->getTorso() != nullptr && agent_perceived->getTorso()->isLocated()){
    if (agent_perceived->getFieldOfView().hasIn(agent_perceived->getTorso()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());
      return true;
    }
  }

  if (agent_perceived->getBase() != nullptr && agent_perceived->getBase()->isLocated()){
    if (agent_perceived->getFieldOfView().hasIn(agent_perceived->getBase()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());      
      return true;
    }
  }
  return false;
}

bool FactsCalculator::isLookingAt(Agent* agent, std::unordered_set<int>& seen_bullet_ids, const Object* object)
{
  if (seen_bullet_ids.count(object->bulletId())){
    facts_.emplace_back(agent->getId(), "isLookingAt", object->id());
    return true;
  }
  return false;
}

} // namespace owds