#include "overworld/Engine/Physics/PhysX/Context.h"

#include <PxBroadPhase.h>
#include <PxSceneDesc.h>
#include <extensions/PxDefaultSimulationFilterShader.h>
#include <foundation/PxVec3.h>
#include <iostream>
#include <pvd/PxPvdSceneClient.h>
#include <string>

#include "overworld/Engine/Physics/PhysX/SharedContext.h"

namespace owds::physx {

  Context::Context(bool minimize_memory_footprint,
                   const std::string& debugger_address) : shared_ctx_(createContext())
  {
    // Create new scene
    (void)debugger_address;
    auto scene_desc = ::physx::PxSceneDesc(shared_ctx_->px_physics_->getTolerancesScale());
    scene_desc.gravity = ::physx::PxVec3(0.0f, 0, -9.81f);
    scene_desc.cpuDispatcher = shared_ctx_->px_dispatcher_.get();
    scene_desc.filterShader = ::physx::PxDefaultSimulationFilterShader;
    scene_desc.cudaContextManager = shared_ctx_->px_cuda_context_manager_.get();
    scene_desc.flags |= ::physx::PxSceneFlag::eENABLE_GPU_DYNAMICS |
                        ::physx::PxSceneFlag::eENABLE_STABILIZATION |
                        ::physx::PxSceneFlag::eENABLE_CCD;
    scene_desc.broadPhaseType = ::physx::PxBroadPhaseType::eGPU;

    px_scene_ = shared_ctx_->px_physics_->createScene(scene_desc);
    // px_scene_->setVisualizationParameter(::physx::PxVisualizationParameter::eSCALE, 0.f);

    if(const auto nv_pvd_client = px_scene_->getScenePvdClient(); nv_pvd_client)
    {
      nv_pvd_client->setScenePvdFlag(::physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
      nv_pvd_client->setScenePvdFlag(::physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
      nv_pvd_client->setScenePvdFlag(::physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    minimize_memory_footprint_ = minimize_memory_footprint;
  }

  Context::~Context()
  {
    if(shared_ctx_->ref_counter_-- <= 1)
      shared_ctx_ = nullptr;
  }

  std::unique_ptr<owds::physx::SharedContext>& Context::createContext()
  {
    static std::unique_ptr<owds::physx::SharedContext> shared_ctx_ = std::make_unique<SharedContext>("0.0.0.0");
    shared_ctx_->ref_counter_++;
    return shared_ctx_;
  }
} // namespace owds::physx