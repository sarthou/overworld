#ifndef OWDS_SITUATIONASSESSOR_H
#define OWDS_SITUATIONASSESSOR_H

#include <atomic>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "overworld/Bullet/PhysicsServers.h"

#include "overworld/Perception/RobotsPerceptionManager.h"
#include "overworld/Perception/ObjectsPerceptionManager.h"
#include "overworld/Perception/HumansPerceptionManager.h"
#include "overworld/Senders/ROSSender.h"
#include "overworld/Facts/FactsCalculator.h"
#include "overworld/Facts/Publisher/OntologeniusFactsPublisher.h"

namespace owds {

class SituationAssessor;

struct HumanAssessor_t
{
  SituationAssessor* assessor;
  std::thread thread;
  PerceptionModuleBase<Object, std::vector<Object*>>* objects_module;
  PerceptionModuleBase<BodyPart, std::vector<BodyPart*>>* humans_module;
  PerceptionModuleBase<BodyPart, std::vector<BodyPart*>>* robots_module;

  HumanAssessor_t()
  {
    assessor = nullptr;
    objects_module = nullptr;
    humans_module = nullptr;
    robots_module = nullptr;
  }
};

class SituationAssessor
{
public:
  SituationAssessor(const std::string& agent_name, bool is_robot = false);
  ~SituationAssessor();

  void run();
  void stop();
  bool isRunning() { return run_; }

  void addObjectPerceptionModule(const std::string& module_name, PerceptionModuleBase_<Object>* module);
  void addHumanPerceptionModule(const std::string& module_name, PerceptionModuleBase_<BodyPart>* module);
  void addRobotPerceptionModule(const std::string& module_name, PerceptionModuleBase_<BodyPart>* module);
  
private:
  std::string agent_name_;
  Agent* myself_agent_;
  bool is_robot_;

  ros::NodeHandle n_;
  ros::CallbackQueue callback_queue_;
  std::atomic<bool> run_;

  BulletClient* bullet_client_;
  RobotsPerceptionManager robots_manager_;
  ObjectsPerceptionManager objects_manager_;
  HumansPerceptionManager humans_manager_;

  FactsCalculator facts_calculator_;
  OntologeniusFactsPublisher facts_publisher_;

  ROSSender* ros_sender_;

  std::map<std::string, HumanAssessor_t> humans_assessors_;

  void assessmentLoop();
  void assess();

  void updateHumansPerspective(const std::string& human_name,
                               const std::map<std::string, Object*>& objects,
                               const std::map<std::string, BodyPart*>& humans,
                               const std::unordered_set<int>& segmented_ids);
  std::map<std::string, HumanAssessor_t>::iterator createHumanAssessor(const std::string& human_name);
};

} // namespace owds

#endif // OWDS_SITUATIONASSESSOR_H