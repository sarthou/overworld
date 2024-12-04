#include "overworld/Physics/Bullet3/Context.h"

#include "overworld/Physics/Bullet3/Actor.h"

namespace owds::bullet3 {
  Context::Context()
  {
    // todo: Figure out if any, and what can be stuffed inside in a shared context
    bt_collision_configuration_ = std::make_unique<btDefaultCollisionConfiguration>();
    bt_collision_dispatcher_ = std::make_unique<btCollisionDispatcher>(bt_collision_configuration_.get());
    bt_broadphase_interface_ = std::make_unique<btDbvtBroadphase>();
    bt_constraint_solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();

    bt_scene_ = std::make_unique<btDiscreteDynamicsWorld>(
      bt_collision_dispatcher_.get(),
      bt_broadphase_interface_.get(),
      bt_constraint_solver_.get(),
      bt_collision_configuration_.get());

    bt_scene_->setGravity(btVector3(0, -9.81, 0));

    bt_ground_geometry_ = std::make_unique<btStaticPlaneShape>(btVector3(0, 1, 0), 0);

    const btVector3 bt_local_inertia(0, 0, 0);
    btRigidBody::btRigidBodyConstructionInfo info(0, nullptr, bt_ground_geometry_.get(), bt_local_inertia);

    bt_ground_actor_ = std::make_unique<btRigidBody>(info);
    bt_ground_actor_->setCollisionFlags(bt_ground_actor_->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    bt_scene_->addRigidBody(bt_ground_actor_.get());
  }

  Context::~Context() noexcept = default;
} // namespace owds::bullet3