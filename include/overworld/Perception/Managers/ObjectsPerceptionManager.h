#ifndef OWDS_ObjectsPerceptionManager_H
#define OWDS_ObjectsPerceptionManager_H

#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"

namespace owds {

  class ObjectsPerceptionManager : public EntitiesPerceptionManager<Object>
  {
  public:
    explicit ObjectsPerceptionManager() : simulate_(false), myself_agent_(nullptr) {}

    const std::map<std::string, Object*>& getEntities() const { return entities_; };

    void setSimulation(bool simulate) { simulate_ = simulate; }
    bool needSimulation() const { return simulated_objects_.size() != 0; }
    void updateSimulatedPoses();

    void setOwnerAgent(Agent* agent) { myself_agent_ = agent; }

  private:
    bool simulate_;
    std::map<std::string, size_t> lost_objects_nb_frames_;
    std::map<std::string, size_t> simulated_objects_;
    std::map<Object*, Pose> goal_poses_;

    std::unordered_set<std::string> false_ids_to_be_merged_;
    std::map<std::string, std::string> merged_ids_;

    Agent* myself_agent_;
    owds::DataFusionBase fusioner_;

    std::map<std::string, Object*>::iterator createFromFusedPercept(Percept<Object>* percept);

    bool handReasoning(Percept<Object>* percept, Object* entity);
    void getPercepts(const std::string& module_name, std::map<std::string, Percept<Object>>& percepts) override;
    bool shouldBeReasonedOn(Object* object);
    void fromfusedToEntities();
    void geometricReasoning();
    void reasoningOnUpdate() override;

    std::map<std::string, Object*> simulatePhysics(const std::map<std::string, Object*>& objects,
                                                   const std::map<std::string, Object*>& objects_to_simulate_oclusion);
    void startSimulation(Object* object);
    void stopSimulation(Object* object, bool erase = true);

    std::vector<PointOfInterest> getPoisInFov(Object* object, Sensor* sensor, std::string module_name);
    bool isObjectInFovAabb(Object* object, Sensor* sensor);

    bool shouldBeSeen(Object* object, Sensor* sensor, const std::vector<PointOfInterest>& pois);

    void mergeFalseIdData();

    void getObjectBoundingBox(Object* object);
  };

} // namespace owds

#endif // OWDS_ObjectsPerceptionManager_H