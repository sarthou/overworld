#ifndef OWDS_PHYSICS_BULLET_CONTEXT_H
#define OWDS_PHYSICS_BULLET_CONTEXT_H

#include <memory>
#include <unordered_map>
#include <vector>

#include "overworld/Physics/Bullet3/API.h"

namespace owds {
  class Joint;
  class Actor;
} // namespace owds

namespace owds::bullet3 {
  class Actor;

  class Context
  {
  public:
    /**
     * I chose std::unordered_map instead of an std::set because its more convenient when dealing with smart pointers.
     */
    std::unordered_map<owds::bullet3::Actor*, std::unique_ptr<owds::bullet3::Actor>> actors_;
    std::unordered_map<owds::Joint*, std::unique_ptr<owds::Joint>> joints_;

    std::vector<std::reference_wrapper<owds::Actor>> actor_list_;

    std::unique_ptr<btCollisionConfiguration> bt_collision_configuration_;
    std::unique_ptr<btCollisionDispatcher> bt_collision_dispatcher_;
    std::unique_ptr<btBroadphaseInterface> bt_broadphase_interface_;
    std::unique_ptr<btConstraintSolver> bt_constraint_solver_;
    std::unique_ptr<btDiscreteDynamicsWorld> bt_scene_;
    std::unique_ptr<btStaticPlaneShape> bt_ground_geometry_;
    std::unique_ptr<btRigidBody> bt_ground_actor_;

    Context();
    ~Context() noexcept;
  };
} // namespace owds::bullet3

#endif // OWDS_PHYSICS_BULLET_CONTEXT_H
