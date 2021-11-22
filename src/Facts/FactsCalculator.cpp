#include "overworld/Facts/FactsCalculator.h"

namespace owds {

FactsCalculator::FactsCalculator(ros::NodeHandle* nh): ontos_(OntologiesManipulator(nh))
{
  ontos_.waitInit();
  ontos_.add("pr2_robot");
  onto_ = ontos_.get("pr2_robot");
  onto_->close();

}

std::vector<Fact> FactsCalculator::computeFacts(const std::map<std::string, Object*>& objects,
                                                const std::map<std::string, Agent*>& agents,
                                                const std::map<std::string, std::unordered_set<int>>& segmantation_ids)
{
  facts_.clear();

  for(auto& obj_from : objects)
  {
    if(obj_from.second->isInHand())
      continue;
    else if(obj_from.second->isStatic())
      continue;

    bool is_in = false;

    for(auto& obj_to : objects)
    {
      if(obj_to.second->isInHand())
        continue;
      else if(obj_to.second->isStatic())
        continue;

      if(obj_from.first != obj_to.first)
        is_in = is_in || isInContainer(obj_to.second, obj_from.second);
    }

    if(is_in == false)
      for(auto& obj_to : objects)
      {
        if(obj_to.second->isInHand())
          continue;
        else if(obj_to.second->isStatic())
          continue;

        if(obj_from.first != obj_to.first)
            isOnTopfOf(obj_to.second, obj_from.second);
      }
  }

  for(auto& agent_from : agents)
  {
    isInHand(agent_from.second);
    for (auto& obj: objects)
    {
      if(obj.second->isStatic())
        continue;

      if (agent_from.second->getType() == AgentType_e::HUMAN){
        hasInHand(agent_from.second, obj.second);
        isHandMovingTowards(agent_from.second, obj.second);
      } 
      auto image_it = segmantation_ids.find(agent_from.first);
      if(image_it != segmantation_ids.end())
        isLookingAt(agent_from.second, image_it->second, obj.second);
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
  else if(object_in->getAabb().min[2] < object_around->getAabb().min[2] - 0.06)
    return false;
  else if(object_in->getAabb().min[2] > object_around->getAabb().max[2] - 0.06)
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
  if(agent->getLeftHand() != nullptr)
    if(agent->getLeftHand()->isEmpty() == false)
    {
      res = true;
      for(auto& object_name : agent->getLeftHand()->getInHand())
        facts_.emplace_back(agent->getId(), "hasInLeftHand", object_name);
    }

  if(agent->getRightHand() != nullptr)
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
    if (agent_perceiving->getFieldOfView().hasIn(agent_perceived->getHead()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());
      return true;
    }
  }

  if (agent_perceived->getLeftHand() != nullptr && agent_perceived->getLeftHand()->isLocated()){
    if (agent_perceiving->getFieldOfView().hasIn(agent_perceived->getLeftHand()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());
      return true;
    }
  }

  if (agent_perceived->getRightHand() != nullptr && agent_perceived->getRightHand()->isLocated()){
    if (agent_perceiving->getFieldOfView().hasIn(agent_perceived->getRightHand()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());
      return true;
    }
  }

  if (agent_perceived->getTorso() != nullptr && agent_perceived->getTorso()->isLocated()){
    if (agent_perceiving->getFieldOfView().hasIn(agent_perceived->getTorso()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());
      return true;
    }
  }

  if (agent_perceived->getBase() != nullptr && agent_perceived->getBase()->isLocated()){
    if (agent_perceiving->getFieldOfView().hasIn(agent_perceived->getBase()->pose().transformIn(head_pose))){
      facts_.emplace_back(agent_perceiving->getId(), "isPerceiving", agent_perceived->getId());      
      return true;
    }
  }
  return false;
}

bool FactsCalculator::isLookingAt(Agent* agent, const std::unordered_set<int>& seen_bullet_ids, const Object* object)
{
  if (seen_bullet_ids.count(object->bulletId())){
    facts_.emplace_back(agent->getId(), "isLookingAt", object->id());
    return true;
  }
  return false;
}

bool FactsCalculator::hasInHand(Agent* agent, Object* object)
{
  Hand *leftHand, *rightHand;
  if ((leftHand = agent->getLeftHand()) != nullptr)
  {
    const auto inLeftHand = leftHand->getInHand();
    if (std::find(inLeftHand.begin(), inLeftHand.end(), object->id()) != inLeftHand.end())
    {
      facts_.emplace_back(agent->getId(), "hasInLeftHand", object->id());
      return true;
    }
    else if (leftHand->isLocated() && object->isLocated())
    {
      if (leftHand->pose().distanceTo(object->pose()) < 0.08)
      {
        std::vector<std::string> types = onto_->individuals.getUp(object->id());
        if (std::find(types.begin(), types.end(), "Pickable") != types.end())
        {
          object->setInHand(leftHand);
          leftHand->putInHand(object);
          facts_.emplace_back(agent->getId(), "hasInLeftHand", object->id());
          return true;
        }
      }
    }
  }

  if ((rightHand = agent->getRightHand()) != nullptr)
  {
    const auto inRightHand = rightHand->getInHand();
    if (std::find(inRightHand.begin(), inRightHand.end(), object->id()) != inRightHand.end())
    {
      facts_.emplace_back(agent->getId(), "hasInRightHand", object->id());
      return true;
    }
    else if (rightHand->isLocated() && object->isLocated())
    {
      if (rightHand->pose().distanceTo(object->pose()) < 0.08)
      {
        std::vector<std::string> types = onto_->individuals.getUp(object->id());
        if (std::find(types.begin(), types.end(), "Pickable") != types.end())
        {
          object->setInHand(rightHand);
          rightHand->putInHand(object);
          facts_.emplace_back(agent->getId(), "hasInRightHand", object->id());
          return true;
        }
      }
    }
  }
  return false;
}

bool FactsCalculator::isHandMovingTowards(Agent* agent, Object* object)
{
  Hand *left_hand, *right_hand;
  bool ret = false;
  if ((left_hand = agent->getLeftHand()) != nullptr)
  {
    const auto in_left_hand = left_hand->getInHand();
    if (std::find(in_left_hand.begin(), in_left_hand.end(), object->id()) != in_left_hand.end())
    {
      return false;
    }
    else if (left_hand->isLocated() && object->isLocated())
    {
      std::array<double, 3> object_to_pose = object->pose().subtractTranslations(left_hand->pose());
      double dist = std::sqrt(object_to_pose[0] * object_to_pose[0] + object_to_pose[1] * object_to_pose[1] + object_to_pose[2] * object_to_pose[2]);
      if (dist < 0.9)
      {
        std::array<double, 3> handSpeed = left_hand->computeTranslationSpeed();
        double handSpeedNorm = std::sqrt(handSpeed[0] * handSpeed[0] + handSpeed[1] * handSpeed[1] + handSpeed[2] * handSpeed[2]);
        if (handSpeedNorm > 0.1)
        {
          double speed_dot_dist = handSpeed[0] * object_to_pose[0] + handSpeed[1] * object_to_pose[1] + handSpeed[2] * object_to_pose[2];
          double speed_to_dist_angle = std::acos(speed_dot_dist / (handSpeedNorm * dist)) * 180. / M_PI;
          if (abs(speed_to_dist_angle) < 10)
          {
            facts_.emplace_back(agent->getId(), "hasLeftHandMovingToward", object->id());
            ret = true;
          }
        }
      }
    }
  }

  if ((right_hand = agent->getRightHand()) != nullptr)
  {
    const auto in_right_hand = right_hand->getInHand();
    if (std::find(in_right_hand.begin(), in_right_hand.end(), object->id()) != in_right_hand.end())
    {
      return false;
    }
    else if (right_hand->isLocated() && object->isLocated())
    {
      std::array<double, 3> object_to_pose = object->pose().subtractTranslations(right_hand->pose());
      double dist = std::sqrt(object_to_pose[0] * object_to_pose[0] + object_to_pose[1] * object_to_pose[1] + object_to_pose[2] * object_to_pose[2]);
      if (dist < 0.9)
      {
        std::array<double, 3> handSpeed = right_hand->computeTranslationSpeed();
        double handSpeedNorm = std::sqrt(handSpeed[0] * handSpeed[0] + handSpeed[1] * handSpeed[1] + handSpeed[2] * handSpeed[2]);
        if (handSpeedNorm > 0.1)
        {
          double speed_dot_dist = handSpeed[0] * object_to_pose[0] + handSpeed[1] * object_to_pose[1] + handSpeed[2] * object_to_pose[2];
          double speed_to_dist_angle = std::acos(speed_dot_dist / (handSpeedNorm * dist)) * 180. / M_PI;
          if (abs(speed_to_dist_angle) < 10)
          {
            facts_.emplace_back(agent->getId(), "hasRightHandMovingToward", object->id());
            ret = true;
          }
        }
      }
    }
  }
  return false;
}

} // namespace owds