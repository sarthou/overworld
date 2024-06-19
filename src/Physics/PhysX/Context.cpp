#include "overworld/Physics/PhysX/Context.h"

#include "overworld/Physics/Base/Joints/Joint.h"
#include "overworld/Physics/PhysX/Actor.h"
#include "overworld/Physics/PhysX/SharedContext.h"

namespace owds::physx {
  std::unique_ptr<owds::physx::SharedContext> Context::shared_ctx_ = nullptr;

  Context::Context(const bool minimize_memory_footprint, const std::string& debugger_address)
  {
    if(shared_ctx_ != nullptr)
    {
      shared_ctx_->ref_counter_++;
    }
    else
    {
      shared_ctx_ = std::make_unique<SharedContext>(debugger_address);
      shared_ctx_->ref_counter_ = 1;
    }

    // Create new scene
    auto sceneDesc = ::physx::PxSceneDesc(shared_ctx_->px_physics_->getTolerancesScale());
    sceneDesc.gravity = ::physx::PxVec3(0.0f, -9.81f, 0.0f);
    sceneDesc.cpuDispatcher = shared_ctx_->px_dispatcher_.get();
    sceneDesc.filterShader = ::physx::PxDefaultSimulationFilterShader;
    sceneDesc.cudaContextManager = shared_ctx_->px_cuda_context_manager_.get();
    sceneDesc.flags |= ::physx::PxSceneFlag::eENABLE_GPU_DYNAMICS;
    sceneDesc.broadPhaseType = ::physx::PxBroadPhaseType::eGPU;

    px_scene_ = shared_ctx_->px_physics_->createScene(sceneDesc);
    // px_scene_->setVisualizationParameter(::physx::PxVisualizationParameter::eSCALE, 0.f);

    if(const auto nvPvdClient = px_scene_->getScenePvdClient(); nvPvdClient)
    {
      nvPvdClient->setScenePvdFlag(::physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
      nvPvdClient->setScenePvdFlag(::physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
      nvPvdClient->setScenePvdFlag(::physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    const auto groundPlane = PxCreatePlane(*shared_ctx_->px_physics_, ::physx::PxPlane(0, 1, 0, 0), *shared_ctx_->px_default_material_);
    px_scene_->addActor(*groundPlane);

    minimize_memory_footprint_ = minimize_memory_footprint;
  }

  Context::~Context()
  {
    if(shared_ctx_->ref_counter_-- <= 1)
    {
      shared_ctx_ = nullptr;
    }
  }
} // namespace owds::physx