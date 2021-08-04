#include "overworld/SituationAssessor.h"

#include "overworld/Perception/Modalities/PR2JointsPerception.h"
#include "overworld/Perception/Modalities/ArTrackPerceptionModule.h"

#include <chrono>
#include <thread>

namespace owds {

SituationAssessor::SituationAssessor(const std::string& agent_name, bool is_robot)
{
  agent_name_ = agent_name;
  is_robot_ = is_robot;

  n_.setCallbackQueue(&callback_queue_);

  if(is_robot_)
    bullet_client_ = PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);
  else
    bullet_client_ = PhysicsServers::connectPhysicsServer(owds::CONNECT_DIRECT);

  robots_manager_.setBulletClient(bullet_client_);
  objects_manager_.setBulletClient(bullet_client_);

  if(is_robot_)
  {
    auto pr2_joint_perception =  new PR2JointsPerception(&n_, agent_name, 
                                                         {{"r_gripper_tool_frame", owds::BODY_PART_RIGHT_HAND}, {"l_gripper_tool_frame", owds::BODY_PART_LEFT_HAND}, {"head_mount_kinect2_rgb_optical_frame", owds::BODY_PART_HEAD}},
                                                         bullet_client_, 0.09);
    robots_manager_.addPerceptionModule("pr2_joints", pr2_joint_perception);
    myself_agent_ = robots_manager_.getAgent(agent_name);

    auto ar_track_perception = new ArTrackPerceptionModule(&n_, myself_agent_);
    objects_manager_.addPerceptionModule("ar_track", ar_track_perception);
  }
}

SituationAssessor::~SituationAssessor()
{
  robots_manager_.deleteModules();
  objects_manager_.deleteModules();
  delete bullet_client_;
}

void SituationAssessor::stop()
{
  run_ = false;
  callback_queue_.disable();
  callback_queue_.clear();
}

void SituationAssessor::run()
{
  std::thread assessment_thread(&SituationAssessor::assessmentLoop, this);

  while(ros::ok())
  {
    callback_queue_.callAvailable(ros::WallDuration(0.1));
  }

  assessment_thread.join();
}

void SituationAssessor::assessmentLoop()
{
  std::chrono::milliseconds interval(90);

  std::chrono::high_resolution_clock::time_point start_time(std::chrono::high_resolution_clock::now());
  std::chrono::high_resolution_clock::time_point next_start_time(start_time);

  while(ros::ok() && isRunning())
  {
    start_time = std::chrono::high_resolution_clock::now();

    assess();

    if(ros::ok() && isRunning())
    {
      next_start_time = start_time + interval;
      std::this_thread::sleep_until(next_start_time);
    }
    
  }
}

void SituationAssessor::assess()
{
  robots_manager_.update();
  objects_manager_.update();
  auto entities = robots_manager_.getEntities();
}

}