#include <overworld/Compat/ROS.h>
// should be first

#include <thread>

#include "overworld/Engine/Common/Camera/Camera.h"
#include "overworld/Engine/Common/Camera/CameraView.h"
#include "overworld/Engine/Common/Models/Loaders/ModelLoader.h"
#include "overworld/Engine/Common/Models/ModelManager.h"
#include "overworld/Engine/Graphics/OpenGL/Renderer.h"

// should be after glad
#include "overworld/Engine/Graphics/GLFW/Window.h"

#if !OWDS_USE_PHYSX
#include <overworld/Physics/Bullet3/Actor.h>
#include <overworld/Physics/Bullet3/World.h>
using DefaultEngine = owds::bullet3::World;
#else
// #include "overworld/Engine/Physics/PhysX/Actor.h"
#include "overworld/Engine/Physics/PhysX/World.h"
using DefaultEngine = owds::physx::World;
#endif

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

// Should be last
#define GLFW_EXPOSE_NATIVE_X11
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

void offscreenThread(DefaultEngine* world, std::vector<int> camera_ids)
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

void worldThread(const std::string& world_name, owds::Window* window)
{
  std::cout << world_name << std::endl;
  auto& cam = window->getCamera();
  cam.setFieldOfView(60.f);
  cam.setOutputAA(owds::ViewAntiAliasing_e::msaa_x4);
  cam.setOutputResolution({640, 480});
  cam.setPositionAndLookAt({6, 6, 1.7}, {0, 0, 0});
  cam.setPlanes({0.1, 60.});
  cam.finalize();

  owds::Renderer renderer;
  renderer.initialize(*window);

  DefaultEngine world(owds::compat::owds_ros::getShareDirectory("overworld"));

  std::cout << "================== WORLD " << world_name << " CREATED ! ================" << std::endl;

  world.setAmbientLight({48.f, -2.f, 0.f},
                        {1.0f, 0.976f, 0.898f},
                        0.25, 0.4, 0.8);

  world.addPointLight({5.f, 7.0f, 2.9f},
                      {1.0f, 1.0f, 1.0f},
                      0.4, 0.5, 1.0,
                      6.f);

  world.addPointLight({8.5f, 7.0f, 2.9f},
                      {1.0f, 1.0f, 1.0f},
                      0.5, 0.6, 1.0,
                      6.0f);

  world.addPointLight({5.f, 12.0f, 2.9f},
                      {1.0f, 1.0f, 1.0f},
                      0.4, 0.5, 1.0,
                      6.f);

  world.addDebugText("overworld", {5, 5, 5}, 0.5, {0., 0.5, 1.0});

  world.addDebugLine({0, 0, 0}, {1, 1, 1});

  auto cam_id = world.addCamera(640, 480, 80, owds::CameraView_e::segmented_view, 0.1, 60.);
  world.setCameraPositionAndLookAt(cam_id, {6, 6, 1.7}, {0, 0, 0});
  std::vector<int> cam_ids = {cam_id};

  renderer.attachWorld(&world);

  std::cout << "================== WORLD ATTACHED ! ================" << std::endl;

  std::string overworld_dir = owds::compat::owds_ros::getShareDirectory("overworld");
  std::cout << "overworld_dir = " << overworld_dir << std::endl;

  //(void)world.loadRobotFromDescription(owds::compat::owds_ros::getShareDirectory("pr2_description") + "/robots/pr2.urdf", false);
  (void)world.createStaticActor(owds::urdf::Geometry_t(overworld_dir + "/models/adream/appartment_vhacd.obj"),
                                {owds::urdf::Geometry_t(overworld_dir + "/models/adream/appartment.obj")},
                                {0., 0., 0.}, {0., 0., 1.57});
  (void)world.createActor(owds::urdf::Geometry_t(overworld_dir + "/models/adream/walls_vhacd.obj"),
                          {owds::urdf::Geometry_t(overworld_dir + "/models/adream/walls.obj")},
                          {0., 0., 0.}, {0., 0., 1.57});
  std::cout << "plop" << std::endl;
  (void)world.loadRobotFromDescription(overworld_dir + "/models/eve.urdf", false);
  //(void)world.loadRobotFromDescription("models/adream/adream.urdf");
  //(void)world.loadRobotFromDescription("models/tutorials/Frame/frame.urdf");
  world.stepSimulation(1.f / 144.f);

  std::cout << "================== WORLD LOADED !! ================" << std::endl;

  std::thread offscreen_thread(offscreenThread, &world, cam_ids);

  while(!window->isCloseRequested())
  {
    window->doPollEvents(renderer);
    world.stepSimulation(1.f / 144.f);
    renderer.commit();
    window->swapBuffer();
  }
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
