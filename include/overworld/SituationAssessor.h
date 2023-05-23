#ifndef OWDS_SITUATIONASSESSOR_H
#define OWDS_SITUATIONASSESSOR_H

#include <atomic>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "overworld/Bullet/PhysicsServers.h"

#include "overworld/Perception/PerceptionManagers.h"

#include "overworld/Senders/ROSSender.h"
#include "overworld/Senders/PoseSender.h"
#include "overworld/Senders/Bernie.h"
#include "overworld/Facts/FactsCalculator.h"
#include "overworld/Facts/Publisher/OntologeniusFactsPublisher.h"

#include <overworld/StartStopModules.h>
#include <overworld/BoundingBox.h>

namespace owds {

class SituationAssessor;

struct HumanAssessor_t
{
  SituationAssessor* assessor;
  std::thread thread;
  PerceptionModuleBase<Object, std::vector<Object*>>* objects_module;
  PerceptionModuleBase<BodyPart, std::vector<BodyPart*>>* humans_module;
  PerceptionModuleBase<BodyPart, std::vector<BodyPart*>>* robots_module;
  PerceptionModuleBase<Area, std::vector<Area*>>* areas_module;

  HumanAssessor_t()
  {
    assessor = nullptr;
    objects_module = nullptr;
    humans_module = nullptr;
    robots_module = nullptr;
    areas_module = nullptr;
  }
};

class SituationAssessor
{
public:
  SituationAssessor(const std::string& agent_name, const std::string& config_path, 
                    double assessment_frequency, double simulation_frequency, 
                    bool simulate = true, bool is_robot = false);
  SituationAssessor(const SituationAssessor& other) = delete;
  ~SituationAssessor();

  void run();
  void stop();
  bool isRunning() { return run_; }

  void addObjectPerceptionModule(const std::string& module_name, PerceptionModuleBase_<Object>* module);
  void addHumanPerceptionModule(const std::string& module_name, PerceptionModuleBase_<BodyPart>* module);
  void addRobotPerceptionModule(const std::string& module_name, PerceptionModuleBase_<BodyPart>* module);
  void addAreaPerceptionModule(const std::string& module_name, PerceptionModuleBase_<Area>* module);
  
private:
  std::string agent_name_;
  Agent* myself_agent_;
  bool is_robot_;

  std::string config_path_;
  bool simulate_;

  ros::NodeHandle n_;
  ros::CallbackQueue callback_queue_;
  ros::ServiceServer start_modules_service_;
  ros::ServiceServer stop_modules_service_;
  ros::ServiceServer bounding_box_service_;
  std::atomic<bool> run_;
  double time_step_; // in second
  double simu_step_;

  BulletClient* bullet_client_;
  PerceptionManagers perception_manager_;

  FactsCalculator facts_calculator_;
  OntologeniusFactsPublisher facts_publisher_;

  ROSSender* ros_sender_;
  PoseSender* motion_planning_pose_sender_;
  BernieSenders* bernie_sender_;

  std::map<std::string, HumanAssessor_t> humans_assessors_;

  void assessmentLoop();
  void assess();

  void processHumans(std::map<std::string, std::unordered_set<int>>& agents_segmentation_ids);
  void updateHumansPerspective(const std::string& human_name,
                               const std::map<std::string, Object*>& objects,
                               const std::map<std::string, BodyPart*>& humans,
                               const std::map<std::string, Area*>& areas,
                               const std::unordered_set<int>& segmented_ids);
  std::map<std::string, HumanAssessor_t>::iterator createHumanAssessor(const std::string& human_name);

  bool stopModules(overworld::StartStopModules::Request &req, overworld::StartStopModules::Response &res);
  bool startModules(overworld::StartStopModules::Request &req, overworld::StartStopModules::Response &res);
  template<typename T>
  bool startModule(BasePerceptionManager<T>& manager, const std::string& module_name, int& status);
  template<typename T>
  bool stopModule(BasePerceptionManager<T>& manager, const std::string& module_name, int& status);
  bool getBoundingBox(overworld::BoundingBox::Request &req, overworld::BoundingBox::Response &res);

};

template<typename T>
bool SituationAssessor::startModule(BasePerceptionManager<T>& manager, const std::string& module_name, int& status)
{
  PerceptionModuleBase_<T>* perception_module = manager.getPerceptionModule(module_name);
  if (perception_module != nullptr)
  {
      if (perception_module->isActivated())
        status = overworld::StartStopModules::Response::ALREADY_ON;
      else
      {
        perception_module->activate(true);
        status = overworld::StartStopModules::Response::OK;
      }
      return true;
  }
  return false;
}

template<typename T>
bool SituationAssessor::stopModule(BasePerceptionManager<T>& manager, const std::string& module_name, int& status)
{
  PerceptionModuleBase_<T>* perception_module = manager.getPerceptionModule(module_name);
  if (perception_module != nullptr)
  {
      if (!perception_module->isActivated())
        status = overworld::StartStopModules::Response::ALREADY_OFF;
      else
      {
        perception_module->activate(false);
        status = overworld::StartStopModules::Response::OK;
      }
      return true;
  }
  return false;
}
} // namespace owds

#endif // OWDS_SITUATIONASSESSOR_H