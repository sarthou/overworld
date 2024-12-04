#include <PxPhysicsAPI.h>
using namespace physx;

int main()
{
  // Initialization
  PxDefaultAllocator allocator;
  PxDefaultErrorCallback errorCallback;
  PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
  PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());
  PxSceneDesc sceneDesc(physics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
  PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(2);
  sceneDesc.cpuDispatcher = dispatcher;
  sceneDesc.filterShader = PxDefaultSimulationFilterShader;
  PxScene* scene = physics->createScene(sceneDesc);
  PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.6f);

  // Create an articulation
  PxArticulation* articulation = physics->createArticulation();

  // Create the base link
  PxTransform basePose(PxVec3(0.0f, 0.5f, 0.0f)); // Position the base at (0, 0.5, 0)
  PxArticulationLink* baseLink = articulation->createLink(nullptr, basePose);
  PxShape* baseShape = physics->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material);
  baseLink->attachShape(*baseShape);
  baseShape->release();
  baseLink->setMass(10.0f);
  baseLink->updateMassAndInertia(*material);

  // Create link 1
  PxTransform link1Pose(PxVec3(0.0f, 1.0f, 0.0f)); // Local pose relative to the base
  PxArticulationLink* link1 = articulation->createLink(baseLink, link1Pose);
  PxShape* link1Shape = physics->createShape(PxBoxGeometry(0.2f, 1.0f, 0.2f), *material);
  link1->attachShape(*link1Shape);
  link1Shape->release();
  link1->setMass(5.0f);
  link1->updateMassAndInertia(*material);

  // Revolute joint between base and link 1
  PxArticulationJoint* joint1 = static_cast<PxArticulationJoint*>(link1->getInboundJoint());
  joint1->setParentPose(PxTransform(PxVec3(0.0f, 0.5f, 0.0f))); // Joint is at the top of the base
  joint1->setChildPose(PxTransform(PxVec3(0.0f, -1.0f, 0.0f))); // Joint connects to the bottom of link 1
  joint1->setJointType(PxArticulationJointType::eREVOLUTE);
  joint1->setAxis(PxVec3(0.0f, 0.0f, 1.0f)); // Rotate around Z-axis

  // Create link 2
  PxTransform link2Pose(PxVec3(0.0f, 1.0f, 0.0f)); // Local pose relative to link 1
  PxArticulationLink* link2 = articulation->createLink(link1, link2Pose);
  PxShape* link2Shape = physics->createShape(PxBoxGeometry(0.2f, 1.0f, 0.2f), *material);
  link2->attachShape(*link2Shape);
  link2Shape->release();
  link2->setMass(3.0f);
  link2->updateMassAndInertia(*material);

  // Revolute joint between link 1 and link 2
  PxArticulationJoint* joint2 = static_cast<PxArticulationJoint*>(link2->getInboundJoint());
  joint2->setParentPose(PxTransform(PxVec3(0.0f, 1.0f, 0.0f))); // Joint is at the top of link 1
  joint2->setChildPose(PxTransform(PxVec3(0.0f, -1.0f, 0.0f))); // Joint connects to the bottom of link 2
  joint2->setJointType(PxArticulationJointType::eREVOLUTE);
  joint2->setAxis(PxVec3(0.0f, 0.0f, 1.0f)); // Rotate around Z-axis

  // Create link 3
  PxTransform link3Pose(PxVec3(0.0f, 1.0f, 0.0f)); // Local pose relative to link 2
  PxArticulationLink* link3 = articulation->createLink(link2, link3Pose);
  PxShape* link3Shape = physics->createShape(PxBoxGeometry(0.2f, 1.0f, 0.2f), *material);
  link3->attachShape(*link3Shape);
  link3Shape->release();
  link3->setMass(2.0f);
  link3->updateMassAndInertia(*material);

  // Revolute joint between link 2 and link 3
  PxArticulationJoint* joint3 = static_cast<PxArticulationJoint*>(link3->getInboundJoint());
  joint3->setParentPose(PxTransform(PxVec3(0.0f, 1.0f, 0.0f))); // Joint is at the top of link 2
  joint3->setChildPose(PxTransform(PxVec3(0.0f, -1.0f, 0.0f))); // Joint connects to the bottom of link 3
  joint3->setJointType(PxArticulationJointType::eREVOLUTE);
  joint3->setAxis(PxVec3(0.0f, 0.0f, 1.0f)); // Rotate around Z-axis

  // Add articulation to the scene
  scene->addArticulation(*articulation);

  // Simulation
  for(int i = 0; i < 100; i++)
  { // Simulate for ~1.67 seconds
    scene->simulate(1.0f / 60.0f);
    scene->fetchResults(true);

    // Print the position of the end effector (Link 3)
    PxTransform link3Pose = link3->getGlobalPose();
    printf("Frame %d: End Effector Position = (%f, %f, %f)\n",
           i, link3Pose.p.x, link3Pose.p.y, link3Pose.p.z);
  }

  // Cleanup
  scene->release();
  articulation->release();
  physics->release();
  foundation->release();

  return 0;
}
