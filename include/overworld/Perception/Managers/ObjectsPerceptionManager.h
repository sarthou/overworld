#ifndef OWDS_ObjectsPerceptionManager_H
#define OWDS_ObjectsPerceptionManager_H

#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"
#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/Object.h"

namespace owds {

class ObjectsPerceptionManager : public EntitiesPerceptionManager<Object>
{
public:
  inline ObjectsPerceptionManager(): EntitiesPerceptionManager(), myself_agent_(nullptr){}
  void setOwnerAgent(Agent* agent) { myself_agent_ = agent; }

  std::map<std::string, Object*> getEntities() const;

  bool needSimulation() { return simulated_objects_.size() != 0; }

private:
  Agent* myself_agent_;
  std::map<std::string, size_t> lost_objects_nb_frames_;
  std::map<std::string, size_t> simulated_objects_;

  std::unordered_set<std::string> false_ids_to_be_merged_;
  std::map<std::string, std::string> merged_ids_;

  std::map<std::string, Object*>::iterator createFromPercept(const Object& percept);

  void getPercepts(std::map<std::string, Object>& percepts) override;
  bool souldBeReasonedOn(Object* object);
  void reasoningOnUpdate() override;

  std::vector<Object*> simulatePhysics(const std::vector<Object*>& objects);
  void startSimulation(Object* object);
  void stopSimulation(Object* object, bool erase = true);

  std::vector<PointOfInterest> getPoisInFov(Object* object);
  bool isObjectsInFovAabb(std::vector<Object*> objects);
  bool shouldBeSeen(Object* object, const std::vector<PointOfInterest>& pois);
  std::unordered_set<int> getObjectsInCamera();

  void mergeFalseIdData();

  void getObjectBoundingBox(Object* object);
};

} // namespace owds

#endif // OWDS_ObjectsPerceptionManager_H