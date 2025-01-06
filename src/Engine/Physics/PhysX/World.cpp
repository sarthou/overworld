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
#include <thread>
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

  void performRaycast(::physx::PxScene* scene,
                      const std::array<float, 3>& origin,
                      const std::array<float, 3>& destination,
                      float max_distance,
                      size_t ray_index,
                      std::vector<RaycastHitResult_t>& results)
  {
    ::physx::PxVec3 origin_vec(origin[0], origin[1], origin[2]);
    ::physx::PxVec3 destination_vec(destination[0], destination[1], destination[2]);

    ::physx::PxVec3 direction = destination_vec - origin_vec;
    ::physx::PxVec3 unit_dir = direction.getNormalized();

    ::physx::PxRaycastBuffer hit_buffer;

    bool hit = scene->raycast(origin_vec, unit_dir, max_distance, hit_buffer);

    RaycastHitResult_t result;
    result.hit = hit;

    if(hit)
    {
      const ::physx::PxRaycastHit& closest_hit = hit_buffer.block;
      result.position = {closest_hit.position.x, closest_hit.position.y, closest_hit.position.z};
      result.normal = {closest_hit.normal.x, closest_hit.normal.y, closest_hit.normal.z};
      result.distance = closest_hit.distance;
      // result.actor_name = (closest_hit.actor != nullptr) ? closest_hit.actor->getName() : "Unnamed Actor";

      ActorData_t* data = static_cast<ActorData_t*>(closest_hit.actor->userData);
      if(data != nullptr)
      {
        result.actor_id = (closest_hit.actor != nullptr) ? data->actor_id : -1;
        result.body_id = (closest_hit.actor != nullptr) ? data->body_id : -1;
      }
    }

    results[ray_index] = result;
  }

  void World::performRaycastsInParallel(const std::vector<std::array<float, 3>>& origins,
                                        const std::vector<std::array<float, 3>>& destinations,
                                        float max_distance,
                                        std::vector<RaycastHitResult_t>& results)
  {
    size_t num_rays = origins.size();
    results.resize(num_rays);

    std::vector<std::thread> threads;
    for(size_t i = 0; i < num_rays; ++i)
      threads.emplace_back(performRaycast, ctx_->px_scene_.get(), origins[i], destinations[i], max_distance, i, std::ref(results));

    for(auto& thread : threads)
      thread.join();
  }

} // namespace owds::physx