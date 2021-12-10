#include "overworld/SituationAssessor.h"

#include "overworld/Perception/Modules/ObjectsModules/ArTrackPerceptionModule.h"
#include "overworld/Perception/Modules/ObjectsModules/Pr2GripperPerceptionModule.h"
#include "overworld/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.h"
#include "overworld/Perception/Modules/ObjectsModules/ObjectsEmulatedPerceptionModule.h"
#include "overworld/Perception/Modules/RobotsModules/PR2JointsPerception.h"
#include "overworld/Perception/Modules/HumansModules/OptitrackPerceptionModule.h"
#include "overworld/Perception/Modules/HumansModules/HumansEmulatedPerceptionModule.h"

#include "overworld/Utility/BulletKeypressHandler.h"

#include <chrono>
#include <thread>

namespace owds {

SituationAssessor::SituationAssessor(const std::string& agent_name, bool is_robot) : facts_publisher_(&n_, agent_name), facts_calculator_(&n_)
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

  /***************************
   * Set perception modules  *
  ***************************/

  auto static_perception = new StaticObjectsPerceptionModule();
  objects_manager_.addPerceptionModule("static", static_perception);

  auto pr2_joint_perception =  new PR2JointsPerception(&n_, "pr2_robot",  // TODO how to avoid fix name ? 
                                                         {{"r_gripper_tool_frame", owds::BODY_PART_RIGHT_HAND}, {"l_gripper_tool_frame", owds::BODY_PART_LEFT_HAND}, {"head_mount_kinect2_rgb_optical_frame", owds::BODY_PART_HEAD}},
                                                         bullet_client_, 0.09);
  robots_manager_.addPerceptionModule("pr2_joints", pr2_joint_perception);

  if(is_robot_)
  {
    myself_agent_ = robots_manager_.getAgent(agent_name);
    
    auto ar_track_perception = new ArTrackPerceptionModule(&n_, myself_agent_);
    objects_manager_.addPerceptionModule("ar_track", ar_track_perception);

    auto left_gripper_perception = new Pr2GripperPerceptionModule(&n_, PR2_GRIPPER_LEFT, bullet_client_, pr2_joint_perception->getPr2BulletId(), myself_agent_);
    objects_manager_.addPerceptionModule("pr2_left_gripper", left_gripper_perception);
    auto right_gripper_perception = new Pr2GripperPerceptionModule(&n_, PR2_GRIPPER_RIGHT, bullet_client_, pr2_joint_perception->getPr2BulletId(), myself_agent_);
    objects_manager_.addPerceptionModule("pr2_right_gripper", right_gripper_perception);

    auto optitrack_perception = new OptitrackPerceptionModule(&n_, "human_0", {6.4868, 2.8506, 0});
    humans_manager_.addPerceptionModule("optitrack", optitrack_perception);
  }
  else
  {
    myself_agent_ = humans_manager_.getAgent(agent_name);
  }

  objects_manager_.setOwnerAgent(myself_agent_);

  ros_sender_ = new ROSSender(&n_);
  if(is_robot_)
  {
    motion_planning_pose_sender_ = new MotionPlanningPoseSender(&n_, objects_manager_);
    bernie_sender_ =  new BernieSenders(&n_);
  }
  else
  {
    motion_planning_pose_sender_ = nullptr;
    bernie_sender_ = nullptr;
  }
  start_modules_service_ = n_.advertiseService(agent_name_ + "/startPerceptionModules", &SituationAssessor::startModules, this);
  stop_modules_service_ = n_.advertiseService(agent_name_ + "/stopPerceptionModules", &SituationAssessor::stopModules, this);
  bounding_box_service_ = n_.advertiseService(agent_name_ + "/getBoundingBox", &SituationAssessor::getBoundingBox, this);
}

SituationAssessor::~SituationAssessor()
{
  robots_manager_.deleteModules();
  objects_manager_.deleteModules();
  humans_manager_.deleteModules();
  
  if(ros_sender_ != nullptr)
    delete ros_sender_;
  if(motion_planning_pose_sender_ != nullptr)
    delete motion_planning_pose_sender_;
  if (bernie_sender_ != nullptr)
    delete bernie_sender_;
  if(bullet_client_ != nullptr)
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

  while(ros::ok() && isRunning())
  {
    callback_queue_.callAvailable(ros::WallDuration(0.1));
  }

  for(auto& human_assessor : humans_assessors_)
  {
    human_assessor.second.assessor->stop();
    human_assessor.second.thread.join();
    delete human_assessor.second.assessor;
  }

  assessment_thread.join();
}

void SituationAssessor::addObjectPerceptionModule(const std::string& module_name, PerceptionModuleBase_<Object>* module)
{
  objects_manager_.addPerceptionModule(module_name, module);
}

void SituationAssessor::addHumanPerceptionModule(const std::string& module_name, PerceptionModuleBase_<BodyPart>* module)
{
  humans_manager_.addPerceptionModule(module_name, module);
}

void SituationAssessor::addRobotPerceptionModule(const std::string& module_name, PerceptionModuleBase_<BodyPart>* module)
{
  robots_manager_.addPerceptionModule(module_name, module);
}

void SituationAssessor::assessmentLoop()
{
  std::chrono::milliseconds interval(60);

  std::chrono::high_resolution_clock::time_point start_time(std::chrono::high_resolution_clock::now());
  std::chrono::high_resolution_clock::time_point next_start_time(start_time);

  while(ros::ok() && isRunning())
  {
    start_time = std::chrono::high_resolution_clock::now();

    assess();

    if(is_robot_)
      handleKeypress(bullet_client_, robots_manager_);

    if(ros::ok() && isRunning())
    {
      if(start_time + interval < std::chrono::high_resolution_clock::now())
      {
        auto delta = std::chrono::high_resolution_clock::now() - (start_time + interval);
        ShellDisplay::warning("[SituationAssessor] [" + agent_name_ + "] The main loop is late of " + std::to_string(delta.count() / 1000000.) + " ms");
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
  std::map<std::string, std::unordered_set<int>> agents_segmentation_ids;

  robots_manager_.update();
  objects_manager_.update();
  humans_manager_.update();
  auto objects = objects_manager_.getEntities();
  auto robots = robots_manager_.getAgents();
  auto humans = humans_manager_.getAgents();
  auto body_parts = humans_manager_.getEntities();

  if(is_robot_)
  {
    for(auto human : humans)
    {
      if(human.second->getHead() == nullptr)
        continue;
      else if(human.second->getHead()->isLocated() == false)
        continue;

      auto proj_matrix = bullet_client_->computeProjectionMatrix(human.second->getFieldOfView().getHeight(),
                                                                 human.second->getFieldOfView().getRatio(),
                                                                 human.second->getFieldOfView().getClipNear(),
                                                                 human.second->getFieldOfView().getClipFar());
      Pose target_pose = human.second->getHead()->pose() * Pose({1,0,0}, {0,0,0,1});
      auto head_pose_trans = human.second->getHead()->pose().arrays().first;
      auto target_pose_trans = target_pose.arrays().first;
      auto view_matrix = bullet_client_->computeViewMatrix({(float)head_pose_trans[0], (float)head_pose_trans[1], (float)head_pose_trans[2]},
                                                           {(float)target_pose_trans[0], (float)target_pose_trans[1], (float)target_pose_trans[2]},
                                                           {0.,0.,1.});
      auto images = bullet_client_->getCameraImage(175*human.second->getFieldOfView().getRatio(), 175, view_matrix, proj_matrix, owds::BULLET_HARDWARE_OPENGL);

      ros_sender_->sendImage(human.first + "/view", images);
      agents_segmentation_ids[human.first] = bullet_client_->getSegmentationIds(images);
      updateHumansPerspective(human.first, objects, body_parts, agents_segmentation_ids[human.first]);
    }
  }

  auto agents = humans;
  agents.insert(robots.begin(), robots.end());

  auto facts = facts_calculator_.computeFacts(objects, agents, agents_segmentation_ids);
  facts_publisher_.publish(facts);

  if(is_robot_)
  {
    ros_sender_->sendEntitiesToTFAndRViz(myself_agent_->getId() + "/objects_markers", objects);
    //bernie_sender_->sendBernie();
  }
  else
    ros_sender_->sendEntitiesToRViz(myself_agent_->getId() + "/objects_markers", objects);
  ros_sender_->sendEntitiesToRViz(myself_agent_->getId() + "/humans_markers", body_parts);
}

void SituationAssessor::updateHumansPerspective(const std::string& human_name,
                                                const std::map<std::string, Object*>& objects,
                                                const std::map<std::string, BodyPart*>& humans,
                                                const std::unordered_set<int>& segmented_ids)
{
  auto assessor_it = humans_assessors_.find(human_name);
  if(assessor_it == humans_assessors_.end())
    assessor_it = createHumanAssessor(human_name);

  std::vector<Object*> seen_objects;
  for(auto object : objects)
  {
    if(object.second->isStatic() == false)
      if(segmented_ids.find(object.second->bulletId()) != segmented_ids.end())
        seen_objects.push_back(object.second);
  }

  std::vector<BodyPart*> seen_humans;
  for(auto body_part : humans)
  {
    if(body_part.second->getAgentName() == human_name)
      seen_humans.push_back(body_part.second);
    else if(segmented_ids.find(body_part.second->bulletId()) != segmented_ids.end())
      seen_humans.push_back(body_part.second);
  }

  assessor_it->second.objects_module->sendPerception(seen_objects);
  assessor_it->second.humans_module->sendPerception(seen_humans);
}

std::map<std::string, HumanAssessor_t>::iterator SituationAssessor::createHumanAssessor(const std::string& human_name)
{
  auto assessor = humans_assessors_.insert(std::make_pair(human_name, HumanAssessor_t())).first;

  assessor->second.assessor = new SituationAssessor(human_name);
  assessor->second.objects_module = new ObjectsEmulatedPerceptionModule();
  assessor->second.humans_module = new HumansEmulatedPerceptionModule();
  assessor->second.assessor->addObjectPerceptionModule("emulated_objects", assessor->second.objects_module);
  assessor->second.assessor->addHumanPerceptionModule("emulated_humans", assessor->second.humans_module);
  std::thread th(&SituationAssessor::run, assessor->second.assessor);
  assessor->second.thread = std::move(th);

  return assessor;
}

bool SituationAssessor::startModules(overworld::StartStopModules::Request &req, overworld::StartStopModules::Response &res)
{
  res.statuses.resize(req.modules.size());
  for (size_t i=0; i < req.modules.size(); i++)
  {
    const std::string& module_name = req.modules[i];
    if (startModule(objects_manager_, module_name, res.statuses[i]))
      continue;
    if (startModule(robots_manager_, module_name, res.statuses[i]))
      continue;
    if (startModule(humans_manager_, module_name, res.statuses[i]))
      continue;
    res.statuses[i] = overworld::StartStopModules::Response::MODULE_NOT_FOUND;
  }
  return true;
}

bool SituationAssessor::stopModules(overworld::StartStopModules::Request &req, overworld::StartStopModules::Response &res)
{
  res.statuses.resize(req.modules.size());
  for (size_t i=0; i < req.modules.size(); i++)
  {
    const std::string& module_name = req.modules[i];
    if (stopModule(objects_manager_, module_name, res.statuses[i]))
      continue;
    if (stopModule(robots_manager_, module_name, res.statuses[i]))
      continue;
    if (stopModule(humans_manager_, module_name, res.statuses[i]))
      continue;
    res.statuses[i] = overworld::StartStopModules::Response::MODULE_NOT_FOUND;
  }
  return true;
}

bool SituationAssessor::getBoundingBox(overworld::BoundingBox::Request &req, overworld::BoundingBox::Response &res)
{
  auto entities = objects_manager_.getEntities();
  auto object = entities.find(req.object_id);
  if(object != entities.end())
  {
    auto bb = object->second->getBoundingBox();
    res.x = bb[0];
    res.y = bb[1];
    res.z = bb[2];
  }
  return true;
}

}