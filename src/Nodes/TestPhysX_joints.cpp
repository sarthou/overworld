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

  // Create materials
  PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.6f); // static friction, dynamic friction, restitution

  // Create parent actor (kinematic)
  PxTransform parentTransform(PxVec3(0.0f, 10.0f, 0.0f)); // Parent at (0, 10, 0)
  PxRigidDynamic* parentActor = physics->createRigidDynamic(parentTransform);
  PxShape* parentShape = physics->createShape(PxBoxGeometry(1.0f, 1.0f, 1.0f), *material);
  parentActor->attachShape(*parentShape);
  parentShape->release();
  parentActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true); // Make the parent kinematic
  scene->addActor(*parentActor);

  // Create child actor (dynamic)
  PxTransform childTransform(PxVec3(0.0f, 8.0f, 0.0f)); // Child at (0, 8, 0)
  PxRigidDynamic* childActor = physics->createRigidDynamic(childTransform);
  PxShape* childShape = physics->createShape(PxBoxGeometry(1.0f, 1.0f, 1.0f), *material);
  childActor->attachShape(*childShape);
  childShape->release();
  scene->addActor(*childActor);

  // Create a fixed joint between the two actors
  PxFixedJoint* joint = PxFixedJointCreate(*physics, parentActor, PxTransform(PxVec3(0.0f, -1.0f, 0.0f)),
                                           childActor, PxTransform(PxVec3(0.0f, 1.0f, 0.0f)));
  if(!joint)
  {
    printf("Failed to create joint\n");
    return -1;
  }

  // Simulate the scene for a short duration
  scene->simulate(1.0f / 60.0f); // 1 frame at 60Hz
  scene->fetchResults(true);

  // Move the parent actor (kinematic actors require `setKinematicTarget`)
  PxTransform newParentTransform(PxVec3(5.0f, 10.0f, 0.0f)); // Move parent to (5, 10, 0)
  parentActor->setKinematicTarget(newParentTransform);

  // Simulate the scene again to allow changes to propagate
  scene->simulate(1.0f / 60.0f); // Another frame at 60Hz
  scene->fetchResults(true);

  // Retrieve the position of the child actor
  PxTransform childPose = childActor->getGlobalPose();
  printf("Child Actor Position: (%f, %f, %f)\n", childPose.p.x, childPose.p.y, childPose.p.z);

  // Cleanup
  scene->release();
  dispatcher->release();
  physics->release();
  foundation->release();

  return 0;
}
