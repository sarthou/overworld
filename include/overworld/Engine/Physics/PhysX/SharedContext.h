#ifndef OWDS_PHYSICS_PHYSX_SHAREDCONTEXT_H
#define OWDS_PHYSICS_PHYSX_SHAREDCONTEXT_H

#include <memory>
#include <string>
#include <unordered_map>

#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Physics/PhysX/API.h"

namespace owds::physx {
  class SharedContext
  {
  public:
    std::size_t ref_counter_ = 0;
    std::unique_ptr<::physx::PxDefaultAllocator> px_allocator_;
    std::unique_ptr<::physx::PxDefaultErrorCallback> px_error_callback_;
    PxPtr<::physx::PxFoundation> px_foundation_;
    PxPtr<::physx::PxPhysics> px_physics_;
    PxPtr<::physx::PxDefaultCpuDispatcher> px_dispatcher_;
    PxPtr<::physx::PxPvd> px_pvd_;
    PxPtr<::physx::PxPvdTransport> px_pvd_transport_;
    PxPtr<::physx::PxCudaContextManager> px_cuda_context_manager_;
    PxPtr<::physx::PxMaterial> px_default_material_;

    explicit SharedContext(const std::string& debugger_address);
    ~SharedContext() noexcept;
  };
} // namespace owds::physx

#endif // OWDS_PHYSICS_PHYSX_SHAREDCONTEXT_H
