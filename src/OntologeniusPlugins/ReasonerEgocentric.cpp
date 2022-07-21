#include "overworld/OntologeniusPlugins/ReasonerEgocentric.h"
#include "ontologenius/graphical/Display.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerEgocentric::initialize()
{
  std::vector<std::string> computable_properties = {"egocentricGeometricalRelation"};
  for(auto& property : computable_properties)
  {
    auto property_ptr = ontology_->object_property_graph_.findBranch(property);
    if(property_ptr != nullptr)
      computable_properties_.insert(property_ptr);
    else
      Display::warning("[ReasonerEgocentric] property " + property + " has not been found");
  }
}

void ReasonerEgocentric::preReason(const QueryInfo_t& query_info)
{
  if((query_info.query_origin == query_origin_individual) &&
     (query_info.query_type == query_relation))
  {
    ObjectPropertyBranch_t* property_ptr = nullptr;
    if(query_info.predicate != "")
    {
      auto property_ptr = isComputableProperty(query_info.predicate);
      if(property_ptr == nullptr)
        return;
    }

    if(query_info.subject != "")
      if(isInDomain(query_info.subject, property_ptr) == false)
        return;

    if(query_info.object != "")
      if(isInRange(query_info.object, property_ptr) == false)
        return;

    std::cout << "[ReasonerEgocentric] Should reason on " << query_info.subject << " : " << query_info.predicate << " : " << query_info.object << std::endl;
  }
}

std::string ReasonerEgocentric::getName()
{
  return "reasoner egocentric";
}

std::string ReasonerEgocentric::getDesciption()
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

bool ReasonerEgocentric::isInRange(const std::string& indiv, ObjectPropertyBranch_t* property)
{
  auto indiv_ptr = ontology_->individual_graph_.findBranch(indiv);
  if(indiv_ptr == nullptr)
    return false;

  std::unordered_set<ClassBranch_t*> types;
  ontology_->individual_graph_.getUpPtr(indiv_ptr, types);
  if(property != nullptr)
  {
    if(property->ranges_.size() == 0)
      return true;

    for(auto range : property->ranges_)
      if(types.find(range.elem) != types.end())
        return true;
  }
  else
  {
    for(auto computable_property : computable_properties_)
    {
      if(computable_property->ranges_.size() == 0)
        return true;

      for(auto range : computable_property->ranges_)
        if(types.find(range.elem) != types.end())
          return true;
    }
  }

  return false;
}

bool ReasonerEgocentric::isInDomain(const std::string& indiv, ObjectPropertyBranch_t* property)
{
  auto indiv_ptr = ontology_->individual_graph_.findBranch(indiv);
  if(indiv_ptr == nullptr)
    return false;

  std::unordered_set<ClassBranch_t*> types;
  ontology_->individual_graph_.getUpPtr(indiv_ptr, types);
  if(property != nullptr)
  {
    if(property->domains_.size() == 0)
      return true;

    for(auto domain : property->domains_)
      if(types.find(domain.elem) != types.end())
        return true;
  }
  else
  {
    for(auto computable_property : computable_properties_)
    {
      if(computable_property->domains_.size() == 0)
        return true;

      for(auto domain : computable_property->domains_)
        if(types.find(domain.elem) != types.end())
          return true;
    }
  }

  return false;
}

PLUGINLIB_EXPORT_CLASS(ReasonerEgocentric, ReasonerInterface)

} // namespace ontologenius
