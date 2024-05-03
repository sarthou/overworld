#ifndef OWDS_DATAFUSIONBASE_H
#define OWDS_DATAFUSIONBASE_H

#include <algorithm>
#include <map>
#include <unordered_map>
#include <string>
#include <vector>
#include <functional>

#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/Object.h"

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ontologenius/OntologiesManipulator.h>

namespace owds {

template<typename T>
class DataFusionBase
{
public:
  explicit DataFusionBase(){}
  virtual ~DataFusionBase() = default;

  virtual void initialize(const std::string& method_name)
  {
    method_name_ = method_name;
  }
  //Todo: const can't be used because of the reasoning on the hand
  virtual std::unordered_map<std::string, T> fuseData(std::map<std::string, std::vector<T>>& aggregated_);

protected:
  std::string method_name_;
  onto::OntologiesManipulator ontos_;
  onto::OntologyManipulator* onto_;
};

//Todo: const can't be used because of the reasoning on the hand
template<typename T>
std::unordered_map<std::string, T> DataFusionBase<T>::fuseData(std::map<std::string, std::vector<T>>& aggregated_)
{
  std::unordered_map<std::string, T> fusioned_percepts;
  fusioned_percepts.reserve(aggregated_.size()); 

  for(auto& pair: aggregated_)
  {   
    T result = pair.second.front(); 
    std::string entity_id = pair.first; 
    for(auto& obj : pair.second)
    {   
      result.merge(&obj);
    }
      fusioned_percepts.emplace(entity_id, result);
  }
  return fusioned_percepts; 
};

} // namespace owds

#endif // OWDS_DATAFUSIONBASE_H