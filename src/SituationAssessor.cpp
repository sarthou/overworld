#include "overworld/SituationAssessor.h"

#include <chrono>
#include <thread>

#include "overworld/Perception/Modules/AreasModules/AreasEmulatedPerceptionModule.h"
#include "overworld/Perception/Modules/HumansModules/HumansEmulatedPerceptionModule.h"
#include "overworld/Perception/Modules/ObjectsModules/ObjectsEmulatedPerceptionModule.h"
#include "overworld/Utils/BulletKeypressHandler.h"

namespace owds {

  SituationAssessor::SituationAssessor(const std::string& agent_name,
                                       const std::string& config_path,
                                       double assessment_frequency,
                                       double simulation_frequency,
                                       bool simulate,
                                       bool is_robot) : agent_name_(agent_name),
                                                        myself_agent_(nullptr),
                                                        is_robot_(is_robot),
                                                        config_path_(config_path),
                                                        simulate_(simulate),
                                                        time_step_(1.0 / assessment_frequency),
                                                        simu_step_(1.0 / simulation_frequency),
                                                        facts_publisher_(agent_name),
                                                        facts_calculator_(agent_name),
                                                        perception_manager_(&n_)
  {
    n_.setCallbackQueue(&callback_queue_);

    if(is_robot_)
    {
      new_assessor_publisher_ = n_.advertise<std_msgs::String>("/overworld/new_assessor", 5);
      agents_list_service_ = n_.advertiseService("/overworld/getAgents", &SituationAssessor::getAgents, this);
      set_simulation_service_ = n_.advertiseService("/overworld/setSimulation", &SituationAssessor::setSimulation, this);
    }

    if(is_robot_)
    {
      world_client_ = PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);
      world_client_->configureDebugVisualizer(COV_ENABLE_GUI, false);
      world_client_->configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, false);
      world_client_->configureDebugVisualizer(COV_ENABLE_SHADOWS, false);
      world_client_->configureDebugVisualizer(COV_ENABLE_PLANAR_REFLECTION, false);
      world_client_->configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, false);
      world_client_->configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW, false);
      world_client_->configureDebugVisualizer(COV_ENABLE_WIREFRAME, false);
      world_client_->configureDebugVisualizer(COV_ENABLE_RENDERING, true);
    }
    else
      world_client_ = PhysicsServers::connectPhysicsServer(owds::CONNECT_DIRECT);

    world_client_->setGravity(0, 0, -9.81);
    world_client_->setTimeStep(simu_step_);

    perception_manager_.setWorldClient(world_client_);

    /***************************
     * Set perception modules  *
     ***************************/

    perception_manager_.setOwnerAgentName(agent_name);

    if(is_robot_)
    {
      if(perception_manager_.applyConfigurationRobot(config_path_) == false)
        throw std::runtime_error("The configuration of overworld has failed. Please look above for more information.");
      if(perception_manager_.getRobotName() != agent_name)
        throw std::runtime_error("The robot name provided in the launch file is different from the robot perception module.");
      myself_agent_ = perception_manager_.robots_manager_.getAgent(agent_name);
    }
    else
    {
      perception_manager_.applyConfigurationHuman(config_path_);
      myself_agent_ = perception_manager_.humans_manager_.getAgent(agent_name);
      perception_manager_.areas_manager_.undrawAreas();
    }

    perception_manager_.objects_manager_.setOwnerAgent(myself_agent_);
    perception_manager_.objects_manager_.setSimulation(simulate_);

    ros_sender_ = new ROSSender(&n_);
    if(is_robot_)
    {
      objetcs_pose_sender_ = new PoseSender(&n_, perception_manager_.objects_manager_);
      bernie_sender_ = new BernieSenders(&n_);
    }
    else
    {
      objetcs_pose_sender_ = nullptr;
      bernie_sender_ = nullptr;
    }
    start_modules_service_ = n_.advertiseService(agent_name_ + "/startPerceptionModules", &SituationAssessor::startModules, this);
    stop_modules_service_ = n_.advertiseService(agent_name_ + "/stopPerceptionModules", &SituationAssessor::stopModules, this);
    bounding_box_service_ = n_.advertiseService(agent_name_ + "/getBoundingBox", &SituationAssessor::getBoundingBox, this);
    if(is_robot_)
    {
      auto msg = std_msgs::String();
      msg.data = "ADD|" + agent_name_;
      new_assessor_publisher_.publish(msg);
    }
  }

  SituationAssessor::~SituationAssessor()
  {
    if(ros_sender_ != nullptr)
      delete ros_sender_;
    if(objetcs_pose_sender_ != nullptr)
      delete objetcs_pose_sender_;
    if(bernie_sender_ != nullptr)
      delete bernie_sender_;
    if(world_client_ != nullptr)
      delete world_client_;
  }

  void SituationAssessor::stop()
  {
    run_ = false;
    callback_queue_.disable();
    callback_queue_.clear();
  }

  void SituationAssessor::setSimulation(bool simulate)
  {
    simulate_ = simulate;
    perception_manager_.objects_manager_.setSimulation(simulate_);
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
    perception_manager_.objects_manager_.addPerceptionModule(module_name, module);
  }

  void SituationAssessor::addHumanPerceptionModule(const std::string& module_name, PerceptionModuleBase_<BodyPart>* module)
  {
    perception_manager_.humans_manager_.addPerceptionModule(module_name, module);
  }

  void SituationAssessor::addRobotPerceptionModule(const std::string& module_name, PerceptionModuleBase_<BodyPart>* module)
  {
    perception_manager_.robots_manager_.addPerceptionModule(module_name, module);
  }

  void SituationAssessor::addAreaPerceptionModule(const std::string& module_name, PerceptionModuleBase_<Area>* module)
  {
    perception_manager_.areas_manager_.addPerceptionModule(module_name, module);
  }

  void SituationAssessor::assessmentLoop()
  {
    std::chrono::milliseconds interval(int(time_step_ * 1000));

    std::chrono::high_resolution_clock::time_point start_time(std::chrono::high_resolution_clock::now());
    std::chrono::high_resolution_clock::time_point next_start_time(start_time);

    while(ros::ok() && isRunning())
    {
      start_time = std::chrono::high_resolution_clock::now();

      assess();

      if(is_robot_)
        handleKeypress(world_client_, perception_manager_);

      if(ros::ok() && isRunning())
      {
        next_start_time = start_time + interval;

        if(simulate_ && perception_manager_.objects_manager_.needSimulation())
        {
          perception_manager_.objects_manager_.initLerp();

          double time_interval_sec = time_step_;
          if(next_start_time < std::chrono::high_resolution_clock::now())
            time_interval_sec = (std::chrono::high_resolution_clock::now() - start_time).count() / 1000000000.;

          unsigned int nb_step = time_interval_sec / simu_step_;
          for(int i = 0; i < nb_step; i++)
          {
            world_client_->stepSimulation();
            perception_manager_.objects_manager_.stepLerp((double)((i + 1) / (double)nb_step));
          }

          perception_manager_.objects_manager_.updateSimulatedPoses();
        }

        if(next_start_time < std::chrono::high_resolution_clock::now())
        {
          auto delta = std::chrono::high_resolution_clock::now() - (start_time + interval);
          ShellDisplay::warning("[SituationAssessor] [" + agent_name_ + "] The main loop is late of " + std::to_string(delta.count() / 1000000.) + " ms");
        }
        else
        {
          // auto delta = next_start_time - std::chrono::high_resolution_clock::now();
          // ShellDisplay::info("sleep for " + std::to_string(delta.count() / 1000000.) + " ms");
          std::this_thread::sleep_until(next_start_time);
        }
      }
    }
  }

  void SituationAssessor::assess()
  {
    std::map<std::string, std::unordered_set<int>> agents_segmentation_ids;

    perception_manager_.update();
    auto objects = perception_manager_.objects_manager_.getEntities();
    auto robots = perception_manager_.robots_manager_.getAgents();
    auto robot_parts = perception_manager_.robots_manager_.getEntities();
    auto humans = perception_manager_.humans_manager_.getAgents();
    auto body_parts = perception_manager_.humans_manager_.getEntities();
    auto areas = perception_manager_.areas_manager_.getEntities();

    std::thread humans_process;

    if(is_robot_)
      humans_process = std::thread(&SituationAssessor::processHumans, this, std::ref(agents_segmentation_ids));

    auto agents = humans;
    agents.insert(robots.begin(), robots.end());

    facts_calculator_.computeObjectsFacts(objects, true);

    if(is_robot_)
    {
      ros_sender_->sendEntitiesToTFAndRViz(myself_agent_->getId() + "/objects_markers", objects);
      ros_sender_->sendEntitiesToTFAndRViz(myself_agent_->getId() + "/humans_markers", body_parts);
      // bernie_sender_->sendBernie();
    }
    else
    {
      ros_sender_->sendEntitiesToRViz(myself_agent_->getId() + "/objects_markers", objects);
      ros_sender_->sendEntitiesToRViz(myself_agent_->getId() + "/humans_markers", body_parts);
    }

    if(is_robot_)
      humans_process.join();

    facts_calculator_.computeAgentsFacts(objects, agents, agents_segmentation_ids, false);
    facts_calculator_.computeAreasFacts(areas, {}, robot_parts, false);
    auto facts = facts_calculator_.computeAreasFacts(areas, objects, body_parts, false);
    facts_publisher_.publish(facts);
  }

  void SituationAssessor::processHumans(std::map<std::string, std::unordered_set<int>>& agents_segmentation_ids)
  {
    auto objects = perception_manager_.objects_manager_.getEntities();
    auto humans = perception_manager_.humans_manager_.getAgents();
    auto body_parts = perception_manager_.humans_manager_.getEntities();
    auto areas = perception_manager_.areas_manager_.getEntities();

    for(auto human : humans)
    {
      if(human.second->getSensors().empty())
        continue;
      else if(human.second->getHead()->isLocated() == false)
        continue;

      for(const auto& sensor : human.second->getSensors())
      {
        auto proj_matrix = world_client_->computeProjectionMatrix(sensor.second->getFieldOfView().getHeight(),
                                                                  sensor.second->getFieldOfView().getRatioOpenGl(),
                                                                  sensor.second->getFieldOfView().getClipNear(),
                                                                  sensor.second->getFieldOfView().getClipFar());
        Pose target_pose = sensor.second->pose() * Pose({0, 0, 1}, {0, 0, 0, 1});
        auto head_pose_trans = sensor.second->pose().arrays().first;
        auto target_pose_trans = target_pose.arrays().first;
        auto view_matrix = world_client_->computeViewMatrix({(float)head_pose_trans[0], (float)head_pose_trans[1], (float)head_pose_trans[2]},
                                                            {(float)target_pose_trans[0], (float)target_pose_trans[1], (float)target_pose_trans[2]},
                                                            {0., 0., 1.});
        auto images = world_client_->getCameraImage(300 * sensor.second->getFieldOfView().getRatioOpenGl(), 300, view_matrix, proj_matrix, owds::BULLET_HARDWARE_OPENGL);

        ros_sender_->sendImage(human.first + "/view", images);
        agents_segmentation_ids[human.first] = world_client_->getSegmentationIds(images);
        updateHumansPerspective(human.first, objects, body_parts, areas, agents_segmentation_ids[human.first]);

        break; // TODO consider only one sensor per human
      }
    }
  }

  void SituationAssessor::updateHumansPerspective(const std::string& human_name,
                                                  const std::map<std::string, Object*>& objects,
                                                  const std::map<std::string, BodyPart*>& humans,
                                                  const std::map<std::string, Area*>& areas,
                                                  const std::unordered_set<int>& segmented_ids)
  {
    auto assessor_it = humans_assessors_.find(human_name);
    if(assessor_it == humans_assessors_.end())
      assessor_it = createHumanAssessor(human_name);

    std::vector<Object*> seen_objects;
    for(auto object : objects)
    {
      if(object.second->isStatic() == false)
        if(segmented_ids.find(object.second->worldId()) != segmented_ids.end())
          seen_objects.push_back(object.second);
    }

    std::vector<BodyPart*> seen_humans;
    for(auto body_part : humans)
    {
      if(body_part.second->getAgentName() == human_name)
        seen_humans.push_back(body_part.second);
      else if(segmented_ids.find(body_part.second->worldId()) != segmented_ids.end())
        seen_humans.push_back(body_part.second);
    }

    std::vector<Area*> seen_areas;
    std::transform(areas.cbegin(), areas.cend(), std::back_inserter(seen_areas),
                   [](const auto& it) { return it.second; });

    assessor_it->second.humans_module->sendPerception(seen_humans);
    assessor_it->second.objects_module->sendPerception(seen_objects);
    assessor_it->second.areas_module->sendPerception(seen_areas);
  }

  std::map<std::string, HumanAssessor_t>::iterator SituationAssessor::createHumanAssessor(const std::string& human_name)
  {
    auto assessor = humans_assessors_.insert(std::make_pair(human_name, HumanAssessor_t())).first;

    assessor->second.assessor = new SituationAssessor(human_name, config_path_, 1.0 / time_step_, 1.0 / simu_step_, simulate_);
    assessor->second.objects_module = new ObjectsEmulatedPerceptionModule();
    assessor->second.humans_module = new HumansEmulatedPerceptionModule();
    assessor->second.areas_module = new AreasEmulatedPerceptionModule();
    assessor->second.assessor->addObjectPerceptionModule("emulated_objects", assessor->second.objects_module);
    assessor->second.assessor->addHumanPerceptionModule("emulated_humans", assessor->second.humans_module);
    assessor->second.assessor->addAreaPerceptionModule("emulated_areas", assessor->second.areas_module);
    std::thread th(&SituationAssessor::run, assessor->second.assessor);
    assessor->second.thread = std::move(th);
    auto msg = std_msgs::String();
    msg.data = "ADD|" + human_name;
    new_assessor_publisher_.publish(msg);

    return assessor;
  }

  bool SituationAssessor::startModules(overworld::StartStopModules::Request& req, overworld::StartStopModules::Response& res)
  {
    res.statuses.resize(req.modules.size());
    for(size_t i = 0; i < req.modules.size(); i++)
    {
      const std::string& module_name = req.modules[i];
      if(startModule(perception_manager_.objects_manager_, module_name, res.statuses[i]))
        continue;
      if(startModule(perception_manager_.robots_manager_, module_name, res.statuses[i]))
        continue;
      if(startModule(perception_manager_.humans_manager_, module_name, res.statuses[i]))
        continue;
      if(startModule(perception_manager_.areas_manager_, module_name, res.statuses[i]))
        continue;
      res.statuses[i] = overworld::StartStopModules::Response::MODULE_NOT_FOUND;
    }
    return true;
  }

  bool SituationAssessor::stopModules(overworld::StartStopModules::Request& req, overworld::StartStopModules::Response& res)
  {
    res.statuses.resize(req.modules.size());
    for(size_t i = 0; i < req.modules.size(); i++)
    {
      const std::string& module_name = req.modules[i];
      if(stopModule(perception_manager_.objects_manager_, module_name, res.statuses[i]))
        continue;
      if(stopModule(perception_manager_.robots_manager_, module_name, res.statuses[i]))
        continue;
      if(stopModule(perception_manager_.humans_manager_, module_name, res.statuses[i]))
        continue;
      if(stopModule(perception_manager_.areas_manager_, module_name, res.statuses[i]))
        continue;
      res.statuses[i] = overworld::StartStopModules::Response::MODULE_NOT_FOUND;
    }
    return true;
  }

  bool SituationAssessor::getBoundingBox(overworld::BoundingBox::Request& req, overworld::BoundingBox::Response& res)
  {
    auto entities = perception_manager_.objects_manager_.getEntities();
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

  bool SituationAssessor::getAgents(overworld::GetAgents::Request& req, overworld::GetAgents::Response& res)
  {
    (void)req;
    res.agents.push_back(agent_name_);
    for(auto& assessor : humans_assessors_)
      res.agents.push_back(assessor.first);
    return true;
  }

  bool SituationAssessor::setSimulation(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    std::cout << "[SituationAssessor] set simulation to " << (int)(req.data) << std::endl;
    setSimulation(req.data);
    for(auto& assessor : humans_assessors_)
      assessor.second.assessor->setSimulation(req.data);
    res.success = true;
    return true;
  }

} // namespace owds