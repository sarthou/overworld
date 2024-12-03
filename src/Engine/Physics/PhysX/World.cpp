#include "overworld/Engine/Physics/PhysX/World.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <foundation/PxSimpleTypes.h>
#include <foundation/PxVec3.h>
#include <glm/vec3.hpp>
#include <string>
#include <vector>

#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/Joints/Joint.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"
#include "overworld/Engine/Common/World.h"
#include "overworld/Engine/Physics/PhysX/Actors/Actor.h"
#include "overworld/Engine/Physics/PhysX/Actors/DynamicActor.h"
#include "overworld/Engine/Physics/PhysX/Actors/StaticActor.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "overworld/Physics/PhysX/Joints/JointContinuous.h"
#include "overworld/Physics/PhysX/Joints/JointFixed.h"
#include "overworld/Physics/PhysX/Joints/JointFloating.h"
#include "overworld/Physics/PhysX/Joints/JointPlanar.h"
#include "overworld/Physics/PhysX/Joints/JointPrismatic.h"
#include "overworld/Physics/PhysX/Joints/JointRevolute.h"
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

  size_t World::createActor(const owds::Shape& collision_shape, const std::vector<owds::Shape>& visual_shapes)
  {
    owds::physx::Actor* actor = new owds::physx::DynamicActor(*ctx_, collision_shape, visual_shapes);

    actor->setup({0., 0., 0.}, {0., 0., 0., 1.});
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

  void World::setGravity(const std::array<float, 3>& gravity)
  {
    ctx_->px_scene_->setGravity(::physx::PxVec3(
      static_cast<::physx::PxReal>(gravity[0]),
      static_cast<::physx::PxReal>(gravity[1]),
      static_cast<::physx::PxReal>(gravity[2])));
  }

  void World::stepSimulation(const float delta)
  {
    ctx_->px_scene_->simulate(delta);
    ctx_->px_scene_->fetchResults(true);
  }

} // namespace owds::physx