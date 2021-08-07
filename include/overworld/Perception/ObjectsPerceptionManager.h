#ifndef OWDS_ObjectsPerceptionManager_H
#define OWDS_ObjectsPerceptionManager_H

#include "overworld/Perception/EntitiesPerceptionManager.h"
#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/Object.h"

namespace owds {

class ObjectsPerceptionManager : public EntitiesPerceptionManager<Object>
{
public:
  inline ObjectsPerceptionManager(): EntitiesPerceptionManager(), myself_agent_(nullptr){}
  void setOwnerAgent(Agent* agent) { myself_agent_ = agent; }

private:
  Agent* myself_agent_;
  std::map<std::string, size_t> lost_objects_nb_frames_;

  void getPercepts(const std::map<std::string, Object>& percepts) override;
  void reasoningOnUpdate() override;

  std::vector<PointOfInterest> getPoisInFov(Object* object);
  bool shouldBeSeen(Object* object, const std::vector<PointOfInterest>& pois);
  std::unordered_set<int> getObjectsInCamera();
};

} // namespace owds

#endif // OWDS_ObjectsPerceptionManager_H