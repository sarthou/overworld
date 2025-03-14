#ifndef OWDS_DATAFUSIONBASE_H
#define OWDS_DATAFUSIONBASE_H

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "ontologenius/OntologiesManipulator.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/Percept.h"
#include "overworld/Perception/DataFusion/DataFusionBase.h"

namespace owds {

  class DataFusionBase
  {
  public:
    explicit DataFusionBase() {}
    virtual ~DataFusionBase() = default;

    virtual void initialize(const std::string& method_name)
    {
      method_name_ = method_name;
    }

    void fuseData(std::unordered_map<std::string, Percept<Object>*>& fusioned_percepts,
                  std::map<std::string, std::map<std::string, Percept<Object>>>& entities_aggregated_percepts_);

    // Todo: const can't be used because of the reasoning on the hand
    template<typename T>
    void fuseData(std::unordered_map<std::string, Percept<T>*>& fusioned_percepts,
                  std::map<std::string, std::map<std::string, Percept<T>>>& entities_aggregated_percepts_);

  protected:
    std::string method_name_;
    onto::OntologiesManipulator ontos_;
    onto::OntologyManipulator* onto_;
  };

  // Todo: const can't be used because of the reasoning on the hand
  template<typename T>
  void DataFusionBase::fuseData(std::unordered_map<std::string, Percept<T>*>& fusioned_percepts,
                                std::map<std::string, std::map<std::string, Percept<T>>>& entities_aggregated_percepts_)
  {
    for(auto& it : entities_aggregated_percepts_)
    {
      if(it.second.size() == 0)
        continue;

      auto fused_percept_it = fusioned_percepts.find(it.first);
      if(fused_percept_it == fusioned_percepts.end())
        fused_percept_it = fusioned_percepts.emplace(it.first, new Percept<T>(it.second.begin()->second)).first;

      Percept<T>* percept = fused_percept_it->second;
      percept->setSensorId(it.second.begin()->second.getSensorId());
      percept->setModuleName(it.second.begin()->second.getModuleName());
      std::string percept_id = it.first;

      Pose pose_in_hand;
      Pose pose_in_map;
      int nb_frame_unseen = 1000;

      for(auto& inner_it : it.second)
      {
        if(inner_it.second.isLocated() && nb_frame_unseen >= inner_it.second.getNbFrameUnseen())
        {
          // We take the pose of the most recently perceived percept
          pose_in_map = inner_it.second.pose();
          nb_frame_unseen = inner_it.second.getNbFrameUnseen();
        }
      }
      percept->setNbFrameUnseen(nb_frame_unseen);
      for(auto& inner_it : it.second)
        percept->merge(&inner_it.second);
    }
  }
} // namespace owds

#endif // OWDS_DATAFUSIONBASE_H