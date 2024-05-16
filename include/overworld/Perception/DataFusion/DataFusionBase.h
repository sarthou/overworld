#ifndef OWDS_DATAFUSIONBASE_H
#define OWDS_DATAFUSIONBASE_H

#include <string>
#include <vector>
#include <unordered_map>
#include <map>

#include "ontologenius/OntologiesManipulator.h"

#include "overworld/BasicTypes/Percept.h"

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
  virtual std::unordered_map<std::string, Percept<T>> fuseData(std::map<std::string, std::vector<Percept<T>>>& aggregated_);

protected:
  std::string method_name_;
  onto::OntologiesManipulator ontos_;
  onto::OntologyManipulator* onto_;
};

//Todo: const can't be used because of the reasoning on the hand
template<typename T>
std::unordered_map<std::string, Percept<T>> DataFusionBase<T>::fuseData(std::map<std::string, std::vector<Percept<T>>>& aggregated_)
{
  std::unordered_map<std::string, Percept<T>> fusioned_percepts;
  fusioned_percepts.reserve(aggregated_.size()); 

  for(auto& pair: aggregated_)
  {   
    Percept<T> result = pair.second.front(); 
    std::string entity_id = pair.first; 

    for(auto& obj : pair.second)
      result.merge(&obj);
    
    fusioned_percepts.emplace(entity_id, result);
  }
  return fusioned_percepts; 
};

} // namespace owds

#endif // OWDS_DATAFUSIONBASE_H