#include <overworld/Compat/ROS.h>
// should be first

#include "overworld/Engine/Common/Models/Loaders/ModelLoader.h"
#include "overworld/Engine/Common/Models/ModelManager.h"
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
  owds::Renderer renderer;
  auto* cam = renderer.getRenderCamera();
  cam->setCameraView(owds::CameraView_e::segmented_view);
  cam->setProjection(owds::CameraProjection_e::perspective);
  cam->setFieldOfView(80.f);
  cam->setOutputAA(owds::ViewAntiAliasing_e::msaa_x8);
  cam->setOutputResolution({640, 480});
  cam->setPositionAndLookAt({5, 5, 1.7}, {0, 0, 0});
  cam->setPlanes({0.1, 80.});
  cam->finalize();

  owds::Window window;
  renderer.initialize(window);

  DefaultEngine world(owds::compat::owds_ros::getShareDirectory("overworld"));

  std::cout << "================== WORLD CREATED ! ================" << std::endl;

  world.setAmbientLight({0.5f, 0.2f, -0.3f},
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

  renderer.attachWorld(&world);

  std::cout << "================== WORLD ATTACHED ! ================" << std::endl;

  (void)world.loadRobotFromDescription("models/adream/adream.urdf");
  world.stepSimulation(1.f / 144.f);

  std::cout << "================== WORLD LOADED !! ================" << std::endl;

  while(!window.isCloseRequested())
  {
    window.doPollEvents(renderer);
    // world.stepSimulation(1.f / 144.f);
    renderer.commit();
    window.swapBuffer();
  }

  return 0;
}
