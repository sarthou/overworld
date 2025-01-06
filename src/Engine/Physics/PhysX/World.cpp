#include "overworld/Engine/Physics/PhysX/World.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <foundation/PxPhysicsVersion.h>
#include <foundation/PxSimpleTypes.h>
#include <foundation/PxVec3.h>
#include <glm/detail/type_quat.hpp>
#include <glm/ext/vector_float3.hpp>
#include <string>
#include <vector>

#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"
#include "overworld/Engine/Common/World.h"
#include "overworld/Engine/Physics/PhysX/Actors/Actor.h"
#include "overworld/Engine/Physics/PhysX/Actors/DynamicActor.h"
#include "overworld/Engine/Physics/PhysX/Actors/StaticActor.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "overworld/Engine/Physics/PhysX/Urdf.h"
#include "overworld/Utils/GlmMath.h"

namespace owds::physx {

  World::World(const std::filesystem::path& base_assets_path) : owds::World(base_assets_path),
                                                                ctx_(std::make_unique<owds::physx::Context>(false, "0.0.0.0"))
  {}

  World::~World() = default;

  std::string World::getBackendName() const
  {
    return "PhysX";
  }

  std::string World::getFullyQualifiedBackendName() const
  {
    return getBackendName() +
           " " + std::to_string(PX_PHYSICS_VERSION_MAJOR) +
           "." + std::to_string(PX_PHYSICS_VERSION_MINOR) +
           "." + std::to_string(PX_PHYSICS_VERSION_BUGFIX);
  }

  size_t World::createActor(const owds::Shape& collision_shape,
                            const std::vector<owds::Shape>& visual_shapes,
                            const glm::vec3& position,
                            const glm::quat& orientation)
  {
    owds::physx::Actor* actor = new owds::physx::DynamicActor(*ctx_, collision_shape, visual_shapes);

    actor->setup({position.x, position.y, position.z},
                 {orientation.x, orientation.y, orientation.z, orientation.w});
    actors_.emplace(actor->unique_id_, actor);

    return actor->unique_id_;
  }

  size_t World::createStaticActor(const owds::Shape& collision_shape,
                                  const std::vector<owds::Shape>& visual_shapes,
                                  const glm::vec3& position,
                                  const glm::quat& orientation)
  {
    owds::physx::Actor* actor = new owds::physx::StaticActor(*ctx_, collision_shape, visual_shapes);

    actor->setup({position.x, position.y, position.z},
                 {orientation.x, orientation.y, orientation.z, orientation.w});
    actors_.emplace(actor->unique_id_, actor);

    return actor->unique_id_;
  }

  owds::Urdf* World::loadUrdf(const urdf::Urdf_t& model)
  {
    (void)model;
    owds::physx::Urdf* urdf = new owds::physx::Urdf(*ctx_);
    urdf->setup();

    return urdf;
  }

  void World::insertUrdf(owds::Urdf* urdf)
  {
    urdf->finish();
    for(const auto& actor : urdf->links_)
      actors_.emplace(actor.second->unique_id_, actor.second);
  }

  void World::setGravity(const std::array<float, 3>& gravity)
  {
    ctx_->px_scene_->setGravity(::physx::PxVec3(
      static_cast<::physx::PxReal>(gravity[0]),
      static_cast<::physx::PxReal>(gravity[1]),
      static_cast<::physx::PxReal>(gravity[2])));
  }

  void World::stepSimulation(const float delta)
  {
    ctx_->px_scene_->simulate(delta != 0 ? delta : time_step_);

    if(ctx_->px_scene_->checkResults(true))
      ctx_->px_scene_->fetchResults(true);
    else
      std::cout << "error " << std::endl;
  }

} // namespace owds::physx