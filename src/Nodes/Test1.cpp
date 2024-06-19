#include <overworld/Graphics/Assimp/ModelLoader.h>
#include <overworld/Graphics/Base/Camera.h>
#include <overworld/Graphics/Base/ModelManager.h>
#include <overworld/Graphics/BGFX/Renderer.h>
#include <overworld/Graphics/GLFW3/Window.h>

#include <overworld/Physics/Base/Joints/JointPrismatic.h>

#if 0
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

  renderer.createCamera("john", world, [&](owds::Camera& cam) {
    cam.setCameraView(owds::CameraView_e::segmented_view);
    cam.setProjection(owds::CameraProjection_e::perspective);
    cam.setFieldOfView(80.f);
    cam.setOutputAA(owds::ViewAntiAliasing_e::msaa_x8);
    cam.setOutputResolution({ 640, 480 });
    cam.setPositionAndLookAt({ 5, 25, 25 }, { 0, 0, 0 });
  });
  renderer.runSanityChecks();

  printf("Physics Engine: %s\n", world.getFullyQualifiedBackendName().c_str());

  const auto size = 2.f;

  const auto shapeBox = world.createShapeBox({size,size,size});
  const auto shapeCylinder = world.createShapeCylinder(1, 2);

  auto& a = world.createActor(shapeBox);
  a.setPositionAndOrientation({0, 4, 0}, {0, 0, 0});
  a.setPhysicsEnabled(false);

  auto& b = world.createActor(shapeBox);
  b.setPositionAndOrientation({0, 6, 0}, {0, 0, 0});

  auto& c = world.createActor(shapeCylinder);
  c.setPositionAndOrientation({0, 12, 0}, {0, 0, 0});

  auto& d = world.createActor(shapeBox);
  d.setPositionAndOrientation({0, 12, 4}, {0, 0, 0});

  // 1.5708 radians = 90 degrees
  auto& joint = world.createJointPrismatic(
    a, {0.f, 2.f, 0.f}, {0.f, 0.f, 1.5708f},
    b, {0.f, -2.f, 0.f}, {0.f, 0.f, 1.5708f});

  joint.setLimits(0, 20);

  float test = 0;

  while(!window.isCloseRequested())
  {
    test -= 0.01f;

    a.setPositionAndOrientation(
      {sinf(test) * 10.f, sinf(test) * 10, std::cos(test) * 10.f},
      {0, cosf(test), 0});

    window.doPollEvents(renderer);
    world.stepSimulation(1.f / 144.f);
    renderer.commit();
  }

  return 0;
}
