#ifndef OWDS_PHYSICS_PHYSX_CONTEXT_H
#define OWDS_PHYSICS_PHYSX_CONTEXT_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/Physics/PhysX/API.h"

namespace owds {
  class Joint;
  class Actor;
} // namespace owds

namespace owds::physx {
  class Actor;
  class SharedContext;

  /**
   * INTERNAL DATA STRUCTURE
   */
  class Context
  {
  public:
    explicit Context(bool minimize_memory_footprint, const std::string& debugger_address = "0.0.0.0");
    ~Context();

    Context(const Context& other) = delete;
    Context& operator=(const Context& other) = delete;

    Context(Context&& other) noexcept = delete;
    Context& operator=(Context&& other) = delete;

    /**
     * We need a "queue" to limit the amount of actors we insert per physics update because
     * PhysX takes a massive performance hit during this type of operation.
     */
    std::vector<owds::physx::Actor*> queued_actors_registration_;

    /**
     * I chose std::unordered_map instead of an std::set because its more convenient when dealing with smart pointers.
     */
    std::unordered_map<owds::physx::Actor*, std::unique_ptr<owds::physx::Actor>> actors_;
    std::unordered_map<owds::Joint*, std::unique_ptr<owds::Joint>> joints_;

    std::vector<std::reference_wrapper<owds::Actor>> actor_list_;

    PxPtr<::physx::PxScene> px_scene_;

    bool minimize_memory_footprint_;

    static std::unique_ptr<owds::physx::SharedContext> shared_ctx_;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_CONTEXT_H
