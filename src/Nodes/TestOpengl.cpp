#include <overworld/Compat/ROS.h>
// should be first

#include "overworld/Engine/Common/Models/ModelManager.h"
#include "overworld/Engine/Graphics/Assimp/ModelLoader.h"
#include "overworld/Engine/Graphics/OpenGL/Camera.h"
#include "overworld/Engine/Graphics/OpenGL/Renderer.h"

// should be after glad
#include "overworld/Engine/Graphics/GLFW/Window.h"

#if !OWDS_USE_PHYSX
#include <overworld/Physics/Bullet3/Actor.h>
#include <overworld/Physics/Bullet3/World.h>
using DefaultEngine = owds::bullet3::World;
#else
#include <overworld/Physics/PhysX/Actor.h>
#include <overworld/Physics/PhysX/World.h>
using DefaultEngine = owds::physx::World;
#endif

#include <cmath>
#include <iostream>

int main()
{
  {
    auto& mgr = owds::ModelManager::get();
    mgr.setModelLoader<owds::assimp::ModelLoader>();
  }

  owds::Renderer renderer;
  auto* cam = renderer.getRenderCamera();
  cam->setCameraView(owds::CameraView_e::segmented_view);
  cam->setProjection(owds::CameraProjection_e::perspective);
  cam->setFieldOfView(80.f);
  cam->setOutputAA(owds::ViewAntiAliasing_e::msaa_x8);
  cam->setOutputResolution({640, 480});
  cam->setPositionAndLookAt({5, 5, 5}, {0, 0, 0});
  cam->finalize();

  owds::Window window;
  renderer.initialize(window);

  DefaultEngine world(owds::compat::owds_ros::getShareDirectory("overworld"));

  std::cout << "================== WORLD CREATED ================" << std::endl;

  world.setAmbientLight({-0.2f, -1.0f, -0.3f},
                        {1.0f, 0.976f, 0.898f},
                        0.15, 0.35, 1.0);

  world.addPointLight({2.0f, -2.0f, 1.0f},
                      {1.0f, 1.0f, 1.0f},
                      0.2, 0.3, 1.0,
                      10.f);

  world.addPointLight({10.0f, -2.0f, 1.0f},
                      {0.0f, 1.0f, 1.0f},
                      0.2, 0.3, 1.0,
                      7.0f);

  renderer.attachWorld(&world);

  std::cout << "================== WORLD ATTACHED ================" << std::endl;

  (void)world.loadRobotFromDescription("models/adream/adream.urdf");
  world.stepSimulation(1.f / 144.f);

  std::cout << "================== WORLD LOADED ================" << std::endl;

  while(!window.isCloseRequested())
  {
    window.doPollEvents(renderer);
    // world.stepSimulation(1.f / 144.f);
    renderer.commit();
    window.swapBuffer();
  }

  return 0;
}
