#include <overworld/Engine/Common/Urdf/Joints/JointPrismatic.h>
#include <overworld/Graphics/Assimp/ModelLoader.h>
#include <overworld/Graphics/BGFX/Renderer.h>
#include <overworld/Graphics/Base/Camera.h>
#include <overworld/Graphics/GLFW3/Window.h>

#include "overworld/Engine/Common/Models/ModelManager.h"

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
#include <overworld/Compat/ROS.h>

int main()
{
  {
    auto& mgr = owds::ModelManager::get();
    mgr.setModelLoader<owds::assimp::ModelLoader>();
  }

  owds::glfw3::Window window;
  owds::bgfx::Renderer renderer;

  renderer.initialize(window);

  DefaultEngine world(owds::compat::owds_ros::getShareDirectory("overworld"));

  renderer.attachWorld(world);

  // auto& cam = renderer.createCamera("john");
  renderer.runSanityChecks();

  auto* cam = renderer.getRenderCamera();
  cam->setCameraView(owds::CameraView_e::segmented_view);
  cam->setProjection(owds::CameraProjection_e::perspective);
  cam->setFieldOfView(80.f);
  cam->setOutputAA(owds::ViewAntiAliasing_e::msaa_x8);
  cam->setOutputResolution({640, 480});
  cam->setPositionAndLookAt({5, 5, 5}, {0, 0, 0});
  cam->finalize();

  (void)world.loadRobotFromDescription("models/adream/adream.urdf");

  while(!window.isCloseRequested())
  {
    window.doPollEvents(renderer);
    world.stepSimulation(1.f / 144.f);
    renderer.commit();
  }

  return 0;
}
