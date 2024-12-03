#include "overworld/Engine/Physics/PhysX/SharedContext.h"

#include <cassert>
#include <cudamanager/PxCudaContextManager.h>
#include <foundation/PxPhysicsVersion.h>
#include <string>

namespace owds::physx {

  SharedContext::SharedContext(const std::string& debugger_address)
  {
    px_allocator_ = std::make_unique<::physx::PxDefaultAllocator>();
    px_error_callback_ = std::make_unique<::physx::PxDefaultErrorCallback>();
    px_foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, *px_allocator_, *px_error_callback_);

    assert(px_foundation_ && "PxCreateFoundation failed");

    // todo: Nvidia now wants us to use their proprietary OmniPVD plugin along with their Omniverse 3d engine.
    // I mean we can still use these if we really want to but PVD tied to Windows only..
    if(!debugger_address.empty())
    {
      // Connect to PhysX Visual Debugger server
      px_pvd_ = PxCreatePvd(*px_foundation_);
      px_pvd_transport_ = ::physx::PxDefaultPvdSocketTransportCreate(debugger_address.c_str(), 5425, 10);
      px_pvd_->connect(*px_pvd_transport_, ::physx::PxPvdInstrumentationFlag::eALL);
    }

    // Initialize PhysX engine
    px_physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *px_foundation_, ::physx::PxTolerancesScale(), true, px_pvd_.get());
    px_dispatcher_ = ::physx::PxDefaultCpuDispatcherCreate(2);

    PxInitExtensions(*px_physics_, px_pvd_.get());

    const ::physx::PxCudaContextManagerDesc px_cuda_desc{};
    px_cuda_context_manager_ = PxCreateCudaContextManager(*px_foundation_, px_cuda_desc, PxGetProfilerCallback());

    px_default_material_ = px_physics_->createMaterial(0.5f, 0.5f, 0.5f);
  }

  SharedContext::~SharedContext() noexcept = default;

} // namespace owds::physx