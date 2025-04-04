#include "overworld/SituationAssessor.h"

#include <chrono>
#include <cstddef>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "overworld/BasicTypes/Agent.h"
#include "overworld/BasicTypes/Area.h"
#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/Perception/Modules/AreasModules/AreasEmulatedPerceptionModule.h"
#include "overworld/Perception/Modules/HumansModules/HumansEmulatedPerceptionModule.h"
#include "overworld/Perception/Modules/ObjectsModules/ObjectsEmulatedPerceptionModule.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"
#include "overworld/Utils/KeypressHandler.h"

namespace owds {

  SituationAssessor::SituationAssessor(const std::string& agent_name,
                                       const std::string& config_path,
                                       double assessment_frequency,
                                       size_t simulation_substepping,
                                       bool simulate,
                                       bool publish_debug,
                                       double max_fps,
                                       bool is_robot) : agent_name_(agent_name),
                                                        myself_agent_(nullptr),
                                                        is_robot_(is_robot),
                                                        config_path_(config_path),
                                                        simulate_(simulate),
                                                        debug_(publish_debug),
                                                        max_fps_(max_fps),
                                                        time_step_(1.0 / assessment_frequency),
                                                        simulation_substepping_(simulation_substepping),
                                                        perception_manager_(&n_),
                                                        facts_publisher_(agent_name)

  {
    n_.setCallbackQueue(&callback_queue_);
  }

  SituationAssessor::~SituationAssessor()
  {
    delete ros_sender_;
    delete objetcs_pose_sender_;
    delete engine_;
  }

  void SituationAssessor::initWorld(Window* window)
  {
    engine_ = new Engine(agent_name_, window);
    engine_->initView(max_fps_);

    engine_->world.setAmbientLight({43.6f, 1.43f, 115.f},
                                   {1.0f, 0.976f, 0.898f},
                                   0.25, 0.4, 0.8);

    engine_->world.setSubstepping(simulation_substepping_);

    // we could insert this in the world there if needed
    engine_->finalise();
  }

  void SituationAssessor::initAssessor()
  {
    if(is_robot_)
    {
      new_assessor_publisher_ = n_.advertise<std_msgs::String>("/overworld/new_assessor", 5);
      agents_list_service_ = n_.advertiseService("/overworld/getAgents", &SituationAssessor::getAgents, this);
      set_simulation_service_ = n_.advertiseService("/overworld/setSimulation", &SituationAssessor::setSimulation, this);
    }

    engine_->world.setGravity({0, 0, -9.81});
    engine_->world.setTimeStep(time_step_);

    perception_manager_.setWorldClient(&(engine_->world));

    /***************************
     * Set perception modules  *
     ***************************/

    perception_manager_.setOwnerAgentName(agent_name_);

    if(is_robot_)
    {
      if(perception_manager_.applyConfigurationRobot(config_path_) == false)
        throw std::runtime_error("The configuration of overworld has failed. Please look above for more information.");
      if(perception_manager_.getRobotName() != agent_name_)
        throw std::runtime_error("The robot name provided in the launch file is different from the robot perception module.");
      myself_agent_ = perception_manager_.robots_manager_.getAgent(agent_name_);
    }
    else
    {
      perception_manager_.applyConfigurationHuman(config_path_);
      myself_agent_ = perception_manager_.humans_manager_.getAgent(agent_name_);
    }

    perception_manager_.objects_manager_.setOwnerAgent(myself_agent_);
    perception_manager_.objects_manager_.setSimulation(simulate_);

    ros_sender_ = new ROSSender(&n_);
    if(is_robot_)
    {
      objetcs_pose_sender_ = new PoseSender(&n_, perception_manager_.objects_manager_);
    }
    else
    {
      objetcs_pose_sender_ = nullptr;
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

    engine_->setKeyCallback([this](Key_e key, bool pressed) { handleKeypress(key, pressed, this->engine_, this->perception_manager_); });
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

  void SituationAssessor::rosLoop()
  {
    while(ros::ok() && isRunning())
    {
      callback_queue_.callAvailable(ros::WallDuration(0.1));
    }
    engine_->stop();
  }

  void SituationAssessor::run()
  {
    run_ = true;
    std::thread render_thread(&SituationAssessor::rosLoop, this);
    std::thread assessment_thread(&SituationAssessor::assessmentLoop, this);

    engine_->run();

    for(auto& human_assessor : humans_assessors_)
    {
      human_assessor.second.assessor->stop();
      human_assessor.second.thread.join();
      delete human_assessor.second.assessor;
    }

    // assessment_thread.join();
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
      start_time += interval; //= std::chrono::high_resolution_clock::now();
      next_start_time = start_time + interval;

      perception_manager_.update();

      if(simulate_ && perception_manager_.objects_manager_.needSimulation())
      {
        engine_->world.stepSimulation(time_step_);
        perception_manager_.objects_manager_.updateSimulatedPoses();
      }
      else
        engine_->world.stepSimulation(time_step_); // TODO handle that correctly (in case of overtime)

      assess(); // TODO split it, we could do stuff while the simulation is runing or while sleeping

      if(next_start_time < std::chrono::high_resolution_clock::now())
      {
        auto delta = std::chrono::high_resolution_clock::now() - (start_time + interval);
        if(delta > interval)
        {
          int mod = (delta.count() / 1000000) % interval.count();
          start_time += interval * mod;
          next_start_time += interval * mod;
        }
        ShellDisplay::warning("[SituationAssessor] [" + agent_name_ + "] The main loop is late of " + std::to_string(delta.count() / 1000000.) + " ms");
      }
      else
      {
        if(is_robot_)
        {
          auto delta = next_start_time - std::chrono::high_resolution_clock::now();
          // ShellDisplay::info("sleep for " + std::to_string(delta.count() / 1000000.) + " ms");
        }
        std::this_thread::sleep_until(next_start_time);
      }
    }
  }

  void SituationAssessor::assess()
  {
    // perception_manager_.update();
    auto objects = perception_manager_.objects_manager_.getEntities();
    auto robots = perception_manager_.robots_manager_.getAgents();
    auto robot_parts = perception_manager_.robots_manager_.getEntities();
    auto humans = perception_manager_.humans_manager_.getAgents();
    auto body_parts = perception_manager_.humans_manager_.getEntities();
    auto areas = perception_manager_.areas_manager_.getEntities();

    std::thread humans_process;

    if(is_robot_)
      humans_process = std::thread(&SituationAssessor::processHumans, this);

    auto agents = humans;
    agents.insert(robots.begin(), robots.end());

    facts_calculator_.computeObjectsFacts(objects, true);

    if(is_robot_)
    {
      if(debug_)
      {
        ros_sender_->sendEntitiesToTFAndRViz(myself_agent_->getId() + "/objects_markers", objects);
        ros_sender_->sendEntitiesToTFAndRViz(myself_agent_->getId() + "/humans_markers", body_parts);
      }
      else
      {
        ros_sender_->sendEntitiesToTF(objects);
        ros_sender_->sendEntitiesToTF(body_parts);
      }
    }
    else if(debug_)
    {
      ros_sender_->sendEntitiesToRViz(myself_agent_->getId() + "/objects_markers", objects);
      ros_sender_->sendEntitiesToRViz(myself_agent_->getId() + "/humans_markers", body_parts);
    }

    if(is_robot_)
      humans_process.join();

    facts_calculator_.computeAgentsFacts(objects, agents, agents_segmentation_ids_, false);
    facts_calculator_.initAreas(areas);
    facts_calculator_.computeAreasFacts(areas, {}, robot_parts, false);
    auto facts = facts_calculator_.computeAreasFacts(areas, objects, body_parts, false);
    facts_publisher_.publish(facts);
  }

  void SituationAssessor::processHumans()
  {
    auto humans = perception_manager_.humans_manager_.getAgents();
    if(humans.empty())
      return;

    auto objects = perception_manager_.objects_manager_.getEntities();
    auto body_parts = perception_manager_.humans_manager_.getEntities();
    auto areas = perception_manager_.areas_manager_.getEntities();

    std::vector<int> cameras;
    std::unordered_map<Agent*, int> agent_to_segmentation;
    std::unordered_map<Agent*, int> agent_to_rgba;

    for(auto& human : humans)
    {
      if(human.second->getSensors().empty())
        continue;
      else if(human.second->getHead()->isLocated() == false)
        continue;

      for(const auto& sensor : human.second->getSensors())
      {
        if(sensor.second->isLocated() == false)
          continue;

        auto head_pose_array = sensor.second->pose().arrays();

        if(sensor.second->getWorldSegmentationId() == -1)
        {
          auto fov = sensor.second->getFieldOfView();
          int id = engine_->world.addCamera(300 * fov.getRatioOpenGl(), 300, fov.getRatio(), CameraView_e::segmented_view, fov.getClipNear(), fov.getClipFar());
          sensor.second->setWorldSegmentationId(id);
        }
        int cam_id = sensor.second->getWorldSegmentationId();
        cameras.push_back(cam_id);
        agent_to_segmentation.emplace(human.second, cam_id);

        engine_->world.setCameraPositionAndOrientation(cam_id, head_pose_array.first, head_pose_array.second);

        if(sensor.second->getWorldRgbaId() == -1)
        {
          auto fov = sensor.second->getFieldOfView();
          int id = engine_->world.addCamera(300 * fov.getRatioOpenGl(), 300, fov.getRatio(), CameraView_e::regular_view, fov.getClipNear(), fov.getClipFar());
          sensor.second->setWorldRgbaId(id);
        }
        cam_id = sensor.second->getWorldRgbaId();
        cameras.push_back(cam_id);
        agent_to_rgba.emplace(human.second, cam_id);

        engine_->world.setCameraPositionAndOrientation(cam_id, head_pose_array.first, head_pose_array.second);

        break; // TODO consider only one sensor per human
      }
    }

    engine_->world.requestCameraRender(cameras);

    for(auto& human : humans)
    {
      auto cam_it = agent_to_segmentation.find(human.second);
      if(cam_it != agent_to_segmentation.end())
      {
        agents_segmentation_ids_[human.first] = engine_->world.getCameraSementation(cam_it->second);
        updateHumansPerspective(human.first, objects, body_parts, areas, agents_segmentation_ids_[human.first]);
      }

      auto rgba_it = agent_to_rgba.find(human.second);
      if(rgba_it != agent_to_rgba.end())
      {
        unsigned int w, h;
        uint32_t* image_data = nullptr;
        engine_->world.getCameraImage(rgba_it->second, &image_data, w, h);

        ros_sender_->sendImage(human.first + "/view", image_data, w, h);
      }
    }
  }

  void SituationAssessor::updateHumansPerspective(const std::string& human_name,
                                                  const std::map<std::string, Object*>& objects,
                                                  const std::map<std::string, BodyPart*>& humans,
                                                  const std::map<std::string, Area*>& areas,
                                                  const std::unordered_set<uint32_t>& segmented_ids)
  {
    std::shared_lock<std::shared_timed_mutex> lock(humans_assessors_mutex_);
    auto assessor_it = humans_assessors_.find(human_name);
    if(assessor_it == humans_assessors_.end())
    {
      creation_request_(human_name);
      return;
    }

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

  void SituationAssessor::humanAssessorThread(owds::Window* window)
  {
    initWorld(window);
    initAssessor();

    run();
  }

  void SituationAssessor::createHumanAssessor(const std::string& human_name, Window* window)
  {
    std::lock_guard<std::shared_timed_mutex> lock(humans_assessors_mutex_);
    auto h_assessor = humans_assessors_.insert(std::make_pair(human_name, HumanAssessor_t())).first;

    h_assessor->second.assessor = new SituationAssessor(human_name, config_path_, 1.0 / time_step_, simulation_substepping_, simulate_, false, max_fps_);
    h_assessor->second.objects_module = new ObjectsEmulatedPerceptionModule();
    h_assessor->second.humans_module = new HumansEmulatedPerceptionModule();
    h_assessor->second.areas_module = new AreasEmulatedPerceptionModule();
    h_assessor->second.assessor->addObjectPerceptionModule("emulated_objects", h_assessor->second.objects_module);
    h_assessor->second.assessor->addHumanPerceptionModule("emulated_humans", h_assessor->second.humans_module);
    h_assessor->second.assessor->addAreaPerceptionModule("emulated_areas", h_assessor->second.areas_module);
    std::thread th(&SituationAssessor::humanAssessorThread, h_assessor->second.assessor, window);
    h_assessor->second.thread = std::move(th);
    auto msg = std_msgs::String();
    msg.data = "ADD|" + human_name;
    new_assessor_publisher_.publish(msg);
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
    std::lock_guard<std::shared_timed_mutex> lock(humans_assessors_mutex_);
    for(auto& assessor : humans_assessors_)
      res.agents.push_back(assessor.first);
    return true;
  }

  bool SituationAssessor::setSimulation(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    std::cout << "[SituationAssessor] set simulation to " << (int)(req.data) << std::endl;
    setSimulation(req.data);
    std::lock_guard<std::shared_timed_mutex> lock(humans_assessors_mutex_);
    for(auto& assessor : humans_assessors_)
      assessor.second.assessor->setSimulation(req.data);
    res.success = true;
    return true;
  }

} // namespace owds