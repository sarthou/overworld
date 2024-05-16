#ifndef OWDS_ObjectsPerceptionManager_H
#define OWDS_ObjectsPerceptionManager_H

#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"
#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/Object.h"

namespace owds {

class ObjectsPerceptionManager : public EntitiesPerceptionManager<Object>
{
public:
  explicit ObjectsPerceptionManager(ros::NodeHandle* nh): EntitiesPerceptionManager(nh), simulate_(false), myself_agent_(nullptr){}

  std::map<std::string, Object*> getEntities() const;

  void setSimulation(bool simulate) { simulate_ = simulate; }
  bool needSimulation() const { return simulated_objects_.size() != 0; }
  void updateSimulatedPoses();
  
  void initLerp();
  void stepLerp(double alpha);

  void setOwnerAgent(Agent* agent) { myself_agent_ = agent; }

private:
  bool simulate_;
  std::map<std::string, size_t> lost_objects_nb_frames_;
  std::map<std::string, size_t> simulated_objects_;
  std::map<Object*, Pose> goal_poses_;

  std::unordered_set<std::string> false_ids_to_be_merged_;
  std::map<std::string, std::string> merged_ids_;

  Agent* myself_agent_;
  //pas sur de si je met un pointeur 
  owds::DataFusionBase<Object> fusioner_;

  std::map<std::string, Object*>::iterator createFromFusedPercept(const Percept<Object>& percept);

  bool HandReasoning(std::pair<const std::string, Percept<Object>>& percept, const std::pair<std::string, Object*>& potential_entity);
  void getPercepts(std::map<std::string, Percept<Object>>& percepts) override;
  bool shouldBeReasonedOn(Object* object);
  void fromfusedToEntities(); 
  void geometricReasoning(); 
  void reasoningOnUpdate() override;

  std::vector<Object*> simulatePhysics(const std::vector<Object*>& objects, const std::vector<Object*>& to_simulate_objetcs);
  void startSimulation(Object* object);
  void stopSimulation(Object* object, bool erase = true);

  std::vector<PointOfInterest> getPoisInFov(Object* object);
  std::vector<Object*> isObjectsInFovAabb(std::vector<Object*> objects);
  bool shouldBeSeen(Object* object, const std::vector<PointOfInterest>& pois);
  std::unordered_set<int> getObjectsInCamera();

  void mergeFalseIdData();

  void getObjectBoundingBox(Object* object);
};

} // namespace owds

#endif // OWDS_ObjectsPerceptionManager_H