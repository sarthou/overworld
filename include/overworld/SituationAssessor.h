#ifndef OWDS_SITUATIONASSESSOR_H
#define OWDS_SITUATIONASSESSOR_H

#include <atomic>
#include <string>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "overworld/Bullet/PhysicsServers.h"

#include "overworld/Perception/RobotsPerceptionManager.h"
#include "overworld/Perception/ObjectsPerceptionManager.h"
#include "overworld/Perception/HumansPerceptionManager.h"
#include "overworld/Senders/ROSSender.h"

namespace owds {

class SituationAssessor
{
public:
  SituationAssessor(const std::string& agent_name, bool is_robot = false);
  ~SituationAssessor();

  void run();
  void stop();
  bool isRunning() { return run_; }
  
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

  ROSSender* ros_sender_;

  void assessmentLoop();
  void assess();
};

} // namespace owds

#endif // OWDS_SITUATIONASSESSOR_H