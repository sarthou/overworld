#ifndef OWDS_ObjectsPerceptionManager_H
#define OWDS_ObjectsPerceptionManager_H

#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"
#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/Object.h"

#include <ontologenius/OntologiesManipulator.h>

namespace owds {

class ObjectsPerceptionManager : public EntitiesPerceptionManager<Object>
{
public:
  explicit ObjectsPerceptionManager(ros::NodeHandle* nh): EntitiesPerceptionManager(), myself_agent_(nullptr), simulate_(false), ontos_(OntologiesManipulator(nh)), onto_(nullptr){}
  void setOwnerAgent(Agent* agent);

  std::map<std::string, Object*> getEntities() const;

  void setSimulation(bool simulate) { simulate_ = simulate; }
  bool needSimulation() const { return simulated_objects_.size() != 0; }
  void updateSimulatedPoses();
  
  void initLerp();
  void stepLerp(double alpha);

private:
  Agent* myself_agent_;
  bool simulate_;
  std::map<std::string, size_t> lost_objects_nb_frames_;
  std::map<std::string, size_t> simulated_objects_;
  std::map<Object*, Pose> goal_poses_;

  std::unordered_set<std::string> false_ids_to_be_merged_;
  std::map<std::string, std::string> merged_ids_;

  OntologiesManipulator ontos_;
  OntologyManipulator* onto_;

  std::map<std::string, Object*>::iterator createFromPercept(const Object& percept);

  void getPercepts(std::map<std::string, Object>& percepts) override;
  bool souldBeReasonedOn(Object* object);
  void reasoningOnUpdate() override;

  std::vector<Object*> simulatePhysics(const std::vector<Object*>& objects, const std::vector<Object*>& to_simulate_objetcs);
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