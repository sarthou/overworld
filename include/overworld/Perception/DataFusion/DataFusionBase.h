#ifndef OWDS_DATAFUSIONBASE_H
#define OWDS_DATAFUSIONBASE_H

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "ontologenius/OntologiesManipulator.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/Percept.h"

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
                  std::map<std::string, std::vector<Percept<Object>>>& aggregated_);

    // Todo: const can't be used because of the reasoning on the hand
    template<typename T>
    void fuseData(std::unordered_map<std::string, Percept<T>*>& fusioned_percepts,
                  std::map<std::string, std::vector<Percept<T>>>& aggregated_);

  protected:
    std::string method_name_;
    onto::OntologiesManipulator ontos_;
    onto::OntologyManipulator* onto_;
  };

  // Todo: const can't be used because of the reasoning on the hand
  template<typename T>
  void DataFusionBase::fuseData(std::unordered_map<std::string, Percept<T>*>& fusioned_percepts,
                                std::map<std::string, std::vector<Percept<T>>>& aggregated_)
  {
    for(auto& pair : aggregated_)
    {
      if(pair.second.size() == 0)
        continue;

      auto fused_percept_it = fusioned_percepts.find(pair.first);
      if(fused_percept_it == fusioned_percepts.end())
        fused_percept_it = fusioned_percepts.emplace(pair.first, new Percept<T>(pair.second.front())).first;

      Percept<T>* percept = fused_percept_it->second;
      std::string percept_id = pair.first;

      Pose pose_in_hand;
      Pose pose_in_map;
      int nb_frame_unseen = 1000;

      for(auto& obj : pair.second)
      {
        if(obj.isLocated() && nb_frame_unseen >= obj.getNbFrameUnseen())
        {
          // We take the pose of the most recently perceived percept
          pose_in_map = obj.pose();
          nb_frame_unseen = obj.getNbFrameUnseen();
        }
      }
      percept->setNbFrameUnseen(nb_frame_unseen);
      for(auto& obj : pair.second)
        percept->merge(&obj);
    }
  };
} // namespace owds

#endif // OWDS_DATAFUSIONBASE_H