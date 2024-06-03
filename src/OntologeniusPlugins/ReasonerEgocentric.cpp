#include "overworld/OntologeniusPlugins/ReasonerEgocentric.h"

#include <algorithm>
#include <pluginlib/class_list_macros.h>

#include "ontologenius/graphical/Display.h"

namespace overworld {

  using namespace ontologenius;

  void ReasonerEgocentric::initialize()
  {
    overworld_client_ = n_.serviceClient<overworld::GetRelations>("/overworld/get_relations/" + agent_name_, true);
    std::vector<std::string> computable_properties = {"egocentricGeometricalProperty"};
    for(auto& property : computable_properties)
    {
      auto property_ptr = ontology_->object_property_graph_.findBranch(property);
      if(property_ptr != nullptr)
        computable_properties_.insert(property_ptr);
      else
        Display::warning("[ReasonerEgocentric] property " + property + " has not been found");
    }
  }

  bool ReasonerEgocentric::preReason(const QueryInfo_t& query_info)
  {
    if((query_info.query_origin == query_origin_individual) &&
       (query_info.query_type == query_relation))
    {
      ObjectPropertyBranch_t* property_ptr = nullptr;
      std::set<ObjectPropertyBranch_t*> to_compute_properties;
      if(query_info.predicate != "")
      {
        property_ptr = isComputableProperty(query_info.predicate);
        if(property_ptr == nullptr)
          return false;
      }

      if(query_info.subject != "")
      {
        std::set<ObjectPropertyBranch_t*> valid_properties = isInDomain(query_info.subject, property_ptr);
        if(valid_properties.size() == 0)
          return false;
        to_compute_properties = valid_properties;
      }

      if(query_info.object != "")
      {
        std::set<ObjectPropertyBranch_t*> valid_properties = isInRange(query_info.object, property_ptr);
        if(valid_properties.size() == 0)
          return false;

        if(to_compute_properties.size() == 0)
          to_compute_properties = valid_properties;
        else
        {
          std::set<ObjectPropertyBranch_t*> intersection;
          std::set_intersection(to_compute_properties.begin(), to_compute_properties.end(),
                                valid_properties.begin(), valid_properties.end(),
                                std::inserter(intersection, intersection.begin()));
          to_compute_properties = intersection;
        }

        if(to_compute_properties.size() == 0)
          return false;
      }

      if(to_compute_properties.size() == 0)
        to_compute_properties.insert(property_ptr);

      auto srv = createRequest(query_info.subject, to_compute_properties, query_info.object);

      for(auto prop : to_compute_properties)
        std::cout << "[ReasonerEgocentric] Should reason on " << query_info.subject << " : " << prop->value() << " : " << query_info.object << std::endl;

      if(call(srv))
        updateOntology(srv.response.to_add, srv.response.to_delete);
    }

    return true;
  }

  std::string ReasonerEgocentric::getName()
  {
    return "reasoner egocentric";
  }

  std::string ReasonerEgocentric::getDescription()
  {
    return "This reasoner is provided by Overworld. It computes egocentric relations on query demand.";
  }

  ObjectPropertyBranch_t* ReasonerEgocentric::isComputableProperty(const std::string& property)
  {
    auto property_ptr = ontology_->object_property_graph_.findBranch(property);
    if(property_ptr != nullptr)
    {
      for(auto computable_property : computable_properties_)
      {
        auto down_properties = ontology_->object_property_graph_.getDownPtrSafe(computable_property);
        if(down_properties.find(property_ptr) != down_properties.end())
          return property_ptr;
      }
    }
    return nullptr;
  }

  std::set<ObjectPropertyBranch_t*> ReasonerEgocentric::isInRange(const std::string& indiv, ObjectPropertyBranch_t* property)
  {
    auto indiv_ptr = ontology_->individual_graph_.findBranch(indiv);
    if(indiv_ptr == nullptr)
      return {};

    std::unordered_set<ClassBranch_t*> types;
    ontology_->individual_graph_.getUpPtr(indiv_ptr, types);
    if(property != nullptr)
    {
      if(property->ranges_.size() == 0)
        return {property};

      for(auto range : property->ranges_)
        if(types.find(range.elem) != types.end())
          return {property};

      return {};
    }
    else
    {
      std::set<ObjectPropertyBranch_t*> res;
      for(auto computable_property : computable_properties_)
      {
        if(computable_property->ranges_.size() == 0)
          res.insert(computable_property);

        for(auto range : computable_property->ranges_)
          if(types.find(range.elem) != types.end())
          {
            res.insert(computable_property);
            break;
          }
      }

      return res;
    }
  }

  std::set<ObjectPropertyBranch_t*> ReasonerEgocentric::isInDomain(const std::string& indiv, ObjectPropertyBranch_t* property)
  {
    auto indiv_ptr = ontology_->individual_graph_.findBranch(indiv);
    if(indiv_ptr == nullptr)
      return {};

    std::unordered_set<ClassBranch_t*> types;
    ontology_->individual_graph_.getUpPtr(indiv_ptr, types);
    if(property != nullptr)
    {
      if(property->domains_.size() == 0)
        return {property};

      for(auto domain : property->domains_)
        if(types.find(domain.elem) != types.end())
          return {property};

      return {};
    }
    else
    {
      std::set<ObjectPropertyBranch_t*> res;
      for(auto computable_property : computable_properties_)
      {
        if(computable_property->domains_.size() == 0)
          res.insert(computable_property);

        for(auto domain : computable_property->domains_)
          if(types.find(domain.elem) != types.end())
          {
            res.insert(computable_property);
            break;
          }
      }

      return res;
    }
  }

  overworld::GetRelations ReasonerEgocentric::createRequest(const std::string& subject, const std::set<ObjectPropertyBranch_t*>& predicates, const std::string& object)
  {
    overworld::GetRelations srv;

    srv.request.origin_id = "ReasonerEgocentric";
    for(auto predicate : predicates)
    {
      overworld::Triplet pattern;
      pattern.subject = subject;
      pattern.predicate = predicate->value();
      pattern.object = object;
      srv.request.patterns.push_back(pattern);
    }

    return srv;
  }

  bool ReasonerEgocentric::call(overworld::GetRelations& srv)
  {
    if(overworld_client_.call(srv))
      return true;
    else
    {
      overworld_client_ = n_.serviceClient<overworld::GetRelations>("/overworld/get_relations/" + agent_name_, true);
      if(overworld_client_.call(srv))
        return true;
      else
        return false;
    }
  }

  void ReasonerEgocentric::updateOntology(const std::vector<overworld::Triplet>& to_add, const std::vector<overworld::Triplet>& to_delete)
  {
    for(auto& triplet : to_delete)
    {
      ontology_->individual_graph_.removeRelation(triplet.subject, triplet.predicate, triplet.object);
      nb_update_++;
    }

    for(auto& triplet : to_add)
    {
      auto branch = ontology_->individual_graph_.findBranch(triplet.subject);
      if(branch != nullptr)
      {
        ontology_->individual_graph_.addRelation(branch, triplet.predicate, triplet.object);
        nb_update_++;
      }
    }
  }

} // namespace overworld

PLUGINLIB_EXPORT_CLASS(overworld::ReasonerEgocentric, ontologenius::ReasonerInterface)