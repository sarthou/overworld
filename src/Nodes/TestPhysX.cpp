#include <PxPhysicsAPI.h>

using namespace physx;

int main()
{
  // Initialize PhysX
  PxDefaultAllocator allocator;
  PxDefaultErrorCallback errorCallback;

  PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
  if(!foundation)
    return -1;

  PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());
  if(!physics)
    return -1;

  // Create a scene
  PxSceneDesc sceneDesc(physics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
  PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(2);
  sceneDesc.cpuDispatcher = dispatcher;
  sceneDesc.filterShader = PxDefaultSimulationFilterShader;
  PxScene* scene = physics->createScene(sceneDesc);

  // Create a material
  PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.6f);

  // Create an articulation
  PxArticulationReducedCoordinate* articulation = physics->createArticulationReducedCoordinate();

  // Create the base link
  PxArticulationLink* baseLink = articulation->createLink(
    nullptr, PxTransform(PxVec3(0.0f, 10.0f, 0.0f)));

  PxShape* baseShape = physics->createShape(PxBoxGeometry(1.0f, 1.0f, 1.0f), *material);
  baseLink->attachShape(*baseShape);
  baseShape->release();

  // Create a child link
  PxArticulationLink* childLink = articulation->createLink(
    baseLink, PxTransform(PxVec3(0.0f, 8.0f, 0.0f)));

  PxShape* childShape = physics->createShape(PxBoxGeometry(1.0f, 1.0f, 1.0f), *material);
  childLink->attachShape(*childShape);
  childShape->release();

  // Configure the joint between base and child links
  PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(childLink->getInboundJoint());
  joint->setJointType(PxArticulationJointType::eREVOLUTE);                      // Revolute joint
  joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED); // Limit motion to rotation

  // Set joint limits (optional)
  // joint->setLimit(PxArticulationAxis::eTWIST, -PxPi / 4, PxPi / 4); // Limit: -45 to +45 degrees

  // Set drive properties
  PxArticulationDrive drive_params(50.f, 5.f, PX_MAX_F32);
  joint->setDriveParams(PxArticulationAxis::eTWIST, drive_params);
  // joint->setDrive(PxArticulationAxis::eTWIST, 50.0f, 5.0f, PX_MAX_F32); // Stiffness, damping, force limit

  // Add the articulation to the scene
  scene->addArticulation(*articulation);

  // Simulate the scene initially
  scene->simulate(1.0f / 60.0f);
  scene->fetchResults(true);

  // Move the child link by setting a target angle for the joint
  const PxReal targetAngle = PxPi / 6; // Target position: 30 degrees
  joint->setDriveTarget(PxArticulationAxis::eTWIST, targetAngle);

  // Simulate the scene for several frames to let the motion propagate
  for(int i = 0; i < 500; i++)
  { // Simulate for ~1.67 seconds
    scene->simulate(1.0f / 60.0f);
    scene->fetchResults(true);

    // Retrieve the global position of the child link
    PxTransform childPose = childLink->getGlobalPose();
    printf("Child Link Position: (%f, %f, %f)\n", childPose.p.x, childPose.p.y, childPose.p.z);
  }

  // Cleanup
  scene->release();
  articulation->release();
  physics->release();
  foundation->release();

  return 0;
}
