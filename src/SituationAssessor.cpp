#include "overworld/SituationAssessor.h"

#include "overworld/Perception/Modalities/ArTrackPerceptionModule.h"
#include "overworld/Perception/Modalities/PR2JointsPerception.h"
#include "overworld/Perception/Modalities/StaticObjectsPerceptionModule.h"
#include "overworld/Perception/Modalities/OptitrackPerceptionModule.h"

#include <chrono>
#include <thread>

namespace owds {

SituationAssessor::SituationAssessor(const std::string& agent_name, bool is_robot)
{
  agent_name_ = agent_name;
  is_robot_ = is_robot;

  n_.setCallbackQueue(&callback_queue_);

  if (is_robot_)
  {
      bullet_client_ = PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);
      bullet_client_->configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, false);
      bullet_client_->configureDebugVisualizer(COV_ENABLE_SHADOWS, false);
      bullet_client_->configureDebugVisualizer(COV_ENABLE_PLANAR_REFLECTION, false);
      bullet_client_->configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, false);
      bullet_client_->configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW, false);
      bullet_client_->configureDebugVisualizer(COV_ENABLE_WIREFRAME, false);
      bullet_client_->configureDebugVisualizer(COV_ENABLE_RENDERING, true);
  }
  else
      bullet_client_ = PhysicsServers::connectPhysicsServer(owds::CONNECT_DIRECT);

  robots_manager_.setBulletClient(bullet_client_);
  objects_manager_.setBulletClient(bullet_client_);
  humans_manager_.setBulletClient(bullet_client_);

  if(is_robot_)
  {
    auto pr2_joint_perception =  new PR2JointsPerception(&n_, agent_name, 
                                                         {{"r_gripper_tool_frame", owds::BODY_PART_RIGHT_HAND}, {"l_gripper_tool_frame", owds::BODY_PART_LEFT_HAND}, {"head_mount_kinect2_rgb_optical_frame", owds::BODY_PART_HEAD}},
                                                         bullet_client_, 0.09);
    robots_manager_.addPerceptionModule("pr2_joints", pr2_joint_perception);
    myself_agent_ = robots_manager_.getAgent(agent_name);

    objects_manager_.setOwnerAgent(myself_agent_);
    auto ar_track_perception = new ArTrackPerceptionModule(&n_, myself_agent_);
    objects_manager_.addPerceptionModule("ar_track", ar_track_perception);
    auto static_perception = new StaticObjectsPerceptionModule();
    //static_perception->activate(false);
    objects_manager_.addPerceptionModule("static", static_perception);

    auto optitrack_perception = new OptitrackPerceptionModule(&n_, "human_0", {6.4868, 2.8506, 0});
    humans_manager_.addPerceptionModule("optitrack", optitrack_perception);

    ros_sender_ = new ROSSender(&n_);
  }
}

SituationAssessor::~SituationAssessor()
{
  robots_manager_.deleteModules();
  objects_manager_.deleteModules();
  humans_manager_.deleteModules();
  delete ros_sender_;
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
  run_ = true;

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
      if(start_time + interval < std::chrono::high_resolution_clock::now())
      {
        auto delta = std::chrono::high_resolution_clock::now() - (start_time + interval);
        ShellDisplay::warning("[SituationAssessor] The main loop is late of " + std::to_string(delta.count() / 1000000.) + " ms");
      }
      else
      {
        next_start_time = start_time + interval;
        //auto delta = next_start_time - std::chrono::high_resolution_clock::now();
        //ShellDisplay::info("sleep for " + std::to_string(delta.count() / 1000000.) + " ms");
        std::this_thread::sleep_until(next_start_time);
      }
    }
    
  }
}

void SituationAssessor::assess()
{
  robots_manager_.update();
  objects_manager_.update();
  humans_manager_.update();
  auto entities = objects_manager_.getEntities();

  ros_sender_->sendEntitiesToTFAndRViz("/rviz_test_markers", entities);
}

}