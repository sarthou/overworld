#include <cmath>
#include <cstddef>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "overworld/Engine/Engine.h"

void offscreenThread(owds::WorldEngine* world, std::vector<int> camera_ids)
{
  usleep(1000000);
  world->requestCameraRender(camera_ids);

  auto model_ids = world->getCameraSementation(camera_ids.front());
  for(auto id : model_ids)
    std::cout << "- " << id << std::endl;

  unsigned int w, h;
  unsigned char* image_data = nullptr;
  world->getCameraImage(camera_ids.front(), (uint32_t**)&image_data, w, h);
  cv::Mat image_as_mat(cv::Size(w, h), CV_8UC4, image_data);

  cv::Mat bgra(cv::Size(w, h), CV_8UC4);
  std::vector<int> from_to{0, 3, 1, 0, 2, 1, 3, 2}; // ABGR->BGRA: 0->3, 1->0, 2->1, 3->2
  cv::mixChannels(image_as_mat, bgra, from_to);
  image_as_mat = bgra;

  cv::flip(image_as_mat, image_as_mat, 0);
  cv::namedWindow("DisplayVector2", cv::WINDOW_AUTOSIZE);
  cv::imshow("DisplayVector2", image_as_mat);
  cv::waitKey();
}

void keyCallback(owds::Key_e key, bool pressed)
{
  std::cout << key << " => " << pressed << std::endl;
}

void worldThread(const std::string& world_name, owds::Window* window)
{
  owds::Engine engine(world_name, window);
  engine.initView();
  engine.setKeyCallback(keyCallback);
  std::cout << world_name << std::endl;

  std::cout << "================== WORLD " << world_name << " CREATED ! ================" << std::endl;

  engine.world.setAmbientLight({48.f, -2.f, 0.f},
                               {1.0f, 0.976f, 0.898f},
                               0.25, 0.4, 0.8);

  engine.world.addPointLight({5.f, 7.0f, 2.9f},
                             {1.0f, 1.0f, 1.0f},
                             0.4, 0.5, 1.0,
                             6.f);

  engine.world.addPointLight({8.5f, 7.0f, 2.9f},
                             {1.0f, 1.0f, 1.0f},
                             0.5, 0.6, 1.0,
                             6.0f);

  engine.world.addPointLight({5.f, 12.0f, 2.9f},
                             {1.0f, 1.0f, 1.0f},
                             0.4, 0.5, 1.0,
                             6.f);

  engine.world.addDebugText("overworld", {5, 5, 5}, 0.5, {0., 0.5, 1.0}, true, 10.);

  engine.world.addDebugLine({0, 0, 0}, {1, 1, 1}, {1., 1., 1.}, 10.);

  auto cam_id = engine.world.addCamera(640, 480, 80, owds::CameraView_e::segmented_view, 0.1, 60.);
  engine.world.setCameraPositionAndLookAt(cam_id, {6, 6, 1.7}, {0, 0, 0});
  std::vector<int> cam_ids = {cam_id};

  engine.finalise();

  std::cout << "================== WORLD ATTACHED ! ================" << std::endl;

  std::string overworld_dir = owds::compat::owds_ros::getShareDirectory("overworld");
  std::cout << "overworld_dir = " << overworld_dir << std::endl;

  //(void)world.createStaticActor(owds::urdf::Geometry_t(overworld_dir + "/models/adream/appartment_vhacd.obj"),
  //                              {owds::urdf::Geometry_t(overworld_dir + "/models/adream/appartment.obj")},
  //                              {0., 0., 0.}, {0., 0., 1.57});
  //(void)world.createActor(owds::urdf::Geometry_t(overworld_dir + "/models/adream/walls_vhacd.obj"),
  //                        {owds::urdf::Geometry_t(overworld_dir + "/models/adream/walls.obj")},
  //                        {0., 0., 0.}, {0., 0., 1.57});

  size_t pr2_id = engine.world.loadUrdf(owds::compat::owds_ros::getShareDirectory("pr2_description") + "/robots/pr2.urdf", {4., 3., 0.}, {0., 0., 0.}, false);
  engine.world.addDebugText("Pr2", {0, 0, 1.5}, 0.5, {0.5, 0., 1.0}, true, 0, -1, 1, 2);
  engine.world.addDebugLine({0, 0, 0}, {1, 1, 1}, {1., 1., 1.}, 0., -1, 1, 2);

  //(void)world.loadUrdf(overworld_dir + "/models/eve.urdf", false);
  //(void)engine.world.loadUrdf("models/adream/adream.urdf", {0., 0., 0.}, {0., 0., 0.});
  //(void)world.loadRobotFromDescription("models/tutorials/Frame/frame.urdf");

  std::cout << "================== WORLD LOADED !! ================" << std::endl;

  std::thread offscreen_thread(offscreenThread, &engine.world, cam_ids);

  std::cout << "pr2 has " << engine.world.getNumJoints(pr2_id) << " joints" << std::endl;
  auto pr2_pose = engine.world.getBasePositionAndOrientation(pr2_id);
  std::cout << "pr2_pose = " << pr2_pose.first[0] << " : " << pr2_pose.first[1] << " : " << pr2_pose.first[2] << std::endl;

  engine.world.setBasePositionAndOrientation(pr2_id, {4., 4., 0.}, {0., 0., 0., 1.});
  // engine.world.setBaseVelocity(pr2_id, {0., 0., 0.}, {0., 0., 0.2});
  engine.world.setJointState(pr2_id, "head_pan_joint", 0.9, -0.4);
  engine.world.stepSimulation();
  pr2_pose = engine.world.getBasePositionAndOrientation(pr2_id);
  std::cout << "pr2_pose = " << pr2_pose.first[0] << " : " << pr2_pose.first[1] << " : " << pr2_pose.first[2] << std::endl;

  auto ray_res = engine.world.raycasts({
                                         {0., 0., 0.}
  },
                                       {{4., 4., 0.1}}, 10);
  for(auto& res : ray_res)
    std::cout << "- " << res.actor_id << "=" << res.body_id << " : " << res.distance << std::endl;

  auto aabb = engine.world.getAABB(pr2_id, 3);
  auto l_aabb = engine.world.getLocalAABB(pr2_id, 3);

  std::cout << "AABB = " << aabb.min[0] << " : " << aabb.min[1] << " : " << aabb.min[2] << " == " << aabb.max[0] << " : " << aabb.max[1] << " : " << aabb.max[2] << std::endl;
  std::cout << "LAABB = " << l_aabb.min[0] << " : " << l_aabb.min[1] << " : " << l_aabb.min[2] << " == " << l_aabb.max[0] << " : " << l_aabb.max[1] << " : " << l_aabb.max[2] << std::endl;

  (void)engine.world.createVisualActor({owds::urdf::Geometry_t(overworld_dir + "/models/trees/Tree1_.obj")},
                                       {-4., 0., 0.}, {0., 0., 1.57});

  auto overlap = engine.world.getOverlappingObjects(1, 2);
  for(auto over : overlap)
    std::cout << "- " << over << std::endl;

  engine.run();
}

int main()
{
  owds::Renderer::init();

  owds::Window window1("overworld_bob");
  // owds::Window window2("overworld_alice");

  std::thread world1(worldThread, "overworld_bob", &window1);
  // std::thread world2(worldThread, "overworld_alice", &window2);

  while(1)
  {
    owds::Window::pollEvent();
    usleep(1000);
  }

  world1.join();
  // world2.join();

  owds::Renderer::release();

  return 0;
}
