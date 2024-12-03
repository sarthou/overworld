#ifndef OWDS_PHYSICS_PHYSX_CONTEXT_H
#define OWDS_PHYSICS_PHYSX_CONTEXT_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/Engine/Physics/PhysX/API.h"

namespace owds::physx {

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

    PxPtr<::physx::PxScene> px_scene_;

    bool minimize_memory_footprint_;

    static std::unique_ptr<owds::physx::SharedContext>& createContext();

    std::unique_ptr<owds::physx::SharedContext>& shared_ctx_;
  };

} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_CONTEXT_H
