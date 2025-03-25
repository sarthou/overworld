#include "overworld/Senders/RelationsSender.h"

#define DELTA_FRAME 5
#define MIN_DISTANCE 10

namespace owds {

  RelationsSender::RelationsSender(ros::NodeHandle* nh,
                                   const std::string& agent_name,
                                   ObjectsPerceptionManager* objects_manager,
                                   HumansPerceptionManager* humans_manager,
                                   RobotsPerceptionManager* robots_manager) : frames_(0),
                                                                              agent_name_(agent_name),
                                                                              objects_manager_(objects_manager),
                                                                              humans_manager_(humans_manager),
                                                                              robots_manager_(robots_manager),
                                                                              agent_(nullptr)
  {
    get_relations_service_ = nh->advertiseService("/overworld/get_relations/" + agent_name, &RelationsSender::onGetRelationService, this);
  }

  void RelationsSender::getAgent()
  {
    auto agent = robots_manager_->getAgent(agent_name_);
    if(agent == nullptr)
      agent = humans_manager_->getAgent(agent_name_);
    if(agent == nullptr)
      ShellDisplay::error("[RelationsSender] Agent " + agent_name_ + " does not exist. The sender will have no effect.");
    else
    {
      agent_ = agent->getTorso();
      if(agent_ == nullptr)
      {
        agent_ = agent->getBase();
        if(agent_ == nullptr)
        {
          agent_ = agent->getHead();
          if(agent_ == nullptr)
            ShellDisplay::error("[RelationsSender] Agent " + agent_name_ + " has no torso, base nor head. The sender will have no effect.");
        }
      }
    }
  }

  bool RelationsSender::onGetRelationService(overworld::GetRelations::Request& request,
                                             overworld::GetRelations::Response& response)
  {
    getAgent();
    if(agent_ == nullptr)
      return true;

    std::vector<ToCompute_t> should_compute = shouldRecompute(request);
    for(size_t i = 0; i < should_compute.size(); i++)
    {
      if(should_compute[i].subject == "")
      {
        computeRelationOnAll(request.patterns[i], response,
                             (should_compute[i].deitic_relation || should_compute[i].egocentric_relation),
                             (should_compute[i].intrinsic_relation || should_compute[i].egocentric_relation));

        if(should_compute[i].deitic_relation)
          last_use_[request.origin_id].computed_deictic_relation.last_complete_use = frames_;
        else if(should_compute[i].intrinsic_relation)
          last_use_[request.origin_id].computed_intrinsic_relation.last_complete_use = frames_;
        else if(should_compute[i].egocentric_relation)
          last_use_[request.origin_id].computed_egocentric_relation.last_complete_use = frames_;
      }
      else
      {
        computeRelationOnAll(request.patterns[i], response,
                             (should_compute[i].deitic_relation || should_compute[i].egocentric_relation),
                             (should_compute[i].intrinsic_relation || should_compute[i].egocentric_relation));
        if(should_compute[i].deitic_relation)
          last_use_[request.origin_id].computed_deictic_relation.last_indiv_use[should_compute[i].subject] = frames_;
        else if(should_compute[i].intrinsic_relation)
          last_use_[request.origin_id].computed_intrinsic_relation.last_indiv_use[should_compute[i].subject] = frames_;
        else if(should_compute[i].egocentric_relation)
          last_use_[request.origin_id].computed_egocentric_relation.last_indiv_use[should_compute[i].subject] = frames_;
      }
    }

    return true;
  }

  std::vector<ToCompute_t> RelationsSender::shouldRecompute(const overworld::GetRelations::Request& request)
  {
    auto it = last_use_.find(request.origin_id);
    if(it == last_use_.end())
    {
      last_use_[request.origin_id] = ComputedRelations_t();
      return std::vector<ToCompute_t>(request.patterns.size(), true);
    }
    else if(it->second.last_use + DELTA_FRAME < frames_)
      return std::vector<ToCompute_t>(request.patterns.size(), true);
    else
    {
      std::vector<ToCompute_t> res;
      for(auto& pattern : request.patterns)
      {
        ToCompute_t to_compute;
        to_compute.subject = pattern.subject;
        if(pattern.predicate == "egocentricGeometricalProperty")
          to_compute.egocentric_relation = shouldRecompute(pattern.subject, it->second.computed_egocentric_relation);
        else if((pattern.predicate == "isAtRightOf") || (pattern.predicate == "isAtLeftOf") ||
                (pattern.predicate == "isInFrontOf") || (pattern.predicate == "isBehind"))
          to_compute.deitic_relation = shouldRecompute(pattern.subject, it->second.computed_deictic_relation);
        else if(pattern.predicate == "deicticGeometricalProperty")
          to_compute.deitic_relation = shouldRecompute(pattern.object, it->second.computed_deictic_relation);
        else if((pattern.predicate == "isToTheRightOf") || (pattern.predicate == "isToTheLeftOf") ||
                (pattern.predicate == "isAtTheFrontOf") || (pattern.predicate == "isAtTheBack"))
          to_compute.intrinsic_relation = shouldRecompute(pattern.subject, it->second.computed_intrinsic_relation);
        else if(pattern.predicate == "intrinsicGeometricalProperty")
          to_compute.intrinsic_relation = shouldRecompute(pattern.object, it->second.computed_intrinsic_relation);

        res.push_back(to_compute);
      }

      return res;
    }
  }

  bool RelationsSender::shouldRecompute(const std::string& subject, ComputedRelation_t& computed_relation)
  {
    if(subject != "")
    {
      auto indiv_it = computed_relation.last_indiv_use.find(subject);
      if(indiv_it == computed_relation.last_indiv_use.end())
      {
        computed_relation.last_indiv_use[subject] = frames_;
        return true;
      }
      else if(indiv_it->second + DELTA_FRAME < frames_)
      {
        indiv_it->second = frames_;
        return true;
      }
      else
        return false;
    }
    else if(computed_relation.last_complete_use + DELTA_FRAME < frames_)
    {
      computed_relation.last_complete_use = frames_;
      return true;
    }
    else
      return false;
  }

  void RelationsSender::computeRelationOnAll(const overworld::Triplet& pattern, overworld::GetRelations::Response& response, bool deictic, bool intrinsic)
  {
    if(agent_->isLocated() == false)
      return;

    auto objects = objects_manager_->getEntities();
    for(auto object_a_it = objects.begin(); object_a_it != objects.end(); ++object_a_it)
    {
      if(shouldBeTested(object_a_it->second) == false)
        continue;

      std::cout << "test all " << object_a_it->first << std::endl;

      for(auto object_b_it = std::next(object_a_it); object_b_it != objects.end(); ++object_b_it)
      {
        if(deictic)
          computeDeicticRelation(object_a_it->second, object_b_it->second, response);
      }
    }
  }

  void RelationsSender::computeRelationOnOne(const overworld::Triplet& pattern, overworld::GetRelations::Response& response, bool deictic, bool intrinsic)
  {
    if(agent_->isLocated() == false)
      return;

    auto objects = objects_manager_->getEntities();
    auto ref_object_it = objects.find(pattern.subject);
    if(ref_object_it == objects.end())
      return;

    if(shouldBeTested(ref_object_it->second) == false)
      return;

    std::cout << "test one" << ref_object_it->first << std::endl;

    for(auto object_it = objects.begin(); object_it != objects.end(); ++object_it)
    {
      if(ref_object_it != object_it)
      {
        if(deictic)
          computeDeicticRelation(ref_object_it->second, object_it->second, response);
      }
    }
  }

  void RelationsSender::computeDeicticRelation(Object* object_a, Object* object_b, overworld::GetRelations::Response& response)
  {
    if(shouldBeTested(object_b) == false)
      return;

    if(InOverlappingOnZ(object_a, object_b))
    {
      if(isNextTo(object_a, object_b))
      {
        // agent to object_a
        std::pair<double, double> v = {object_a->pose().getX() - agent_->pose().getX(),
                                       object_a->pose().getY() - agent_->pose().getY()};
        // object_a to object_b
        std::pair<double, double> u = {object_b->pose().getX() - object_a->pose().getX(),
                                       object_b->pose().getY() - object_a->pose().getY()};

        double angle = atan2(u.second, u.first) - atan2(v.second, v.first); // [-pi, +pi]

        overworld::Triplet triplet;
        triplet.subject = object_b->id();
        triplet.object = object_a->id();
        if((angle < M_PI_4) && (angle > -M_PI_4))
          triplet.predicate = "isBehind";
        else if((angle <= 3 * M_PI_4) && (angle >= M_PI_4))
          triplet.predicate = "isAtLeftOf";
        else if((angle >= -3 * M_PI_4) && (angle <= -M_PI_4))
          triplet.predicate = "isAtRightOf";
        else
          triplet.predicate = "isInFrontOf";
        response.to_add.push_back(triplet);
      }
    }
  }

  bool RelationsSender::shouldBeTested(Object* object)
  {
    if(object->isLocated() == false)
      return false;
    else if(object->isStatic())
      return false;
    else if(object->isTrueId() == false)
      return false;
    else if(object->isInHand())
      return false;
    else if(agent_->pose().distanceSqTo(object->pose()) > MIN_DISTANCE)
      return false;
    else
      return true;
  }

  bool RelationsSender::isNextTo(Object* object_a, Object* object_b)
  {
    double max_a = *std::max_element(object_a->getBoundingBox().begin(), object_a->getBoundingBox().end());
    double max_b = *std::max_element(object_b->getBoundingBox().begin(), object_b->getBoundingBox().end());
    double max_size = std::max(max_a, max_b);
    return (object_a->getMinDistanceTo(*object_b) <= max_size * 2.0);
    // we should concider to use the distance between the surfaces of the BBs but it can be stime consuming
  }

  bool RelationsSender::InOverlappingOnZ(Object* object_a, Object* object_b)
  {
    double min_height = std::min(object_a->getAabb().max[2] - object_a->getAabb().min[2], object_b->getAabb().max[2] - object_b->getAabb().min[2]);
    double overlapping_dist = std::max(0.f, std::min(object_a->getAabb().max[2], object_b->getAabb().max[2]) - std::max(object_a->getAabb().min[2], object_b->getAabb().min[2]));
    return (overlapping_dist >= min_height / 3.);
  }

} // namespace owds