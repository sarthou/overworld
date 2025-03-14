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

  void World::setSubstepping(size_t sub_step)
  {
    ctx_->sub_step_ = sub_step;
  }

  size_t World::createActor(const owds::Shape& collision_shape,
                            const std::vector<owds::Shape>& visual_shapes,
                            const std::array<double, 3>& position,
                            const std::array<double, 4>& orientation)
  {
    owds::physx::Actor* actor = new owds::physx::DynamicActor(*ctx_, collision_shape, visual_shapes);
    actor->setup(position, orientation);
    actors_.emplace(actor->unique_id_, actor);

    return actor->unique_id_;
  }

  size_t World::createStaticActor(const owds::Shape& collision_shape,
                                  const std::vector<owds::Shape>& visual_shapes,
                                  const std::array<double, 3>& position,
                                  const std::array<double, 4>& orientation)
  {
    owds::physx::Actor* actor = new owds::physx::StaticActor(*ctx_, collision_shape, visual_shapes);

    actor->setup(position, orientation);
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

  std::unordered_set<int> World::getOverlappingActors(::owds::Actor* owds_actor)
  {
    Actor* actor = static_cast<Actor*>(owds_actor);
    std::unordered_set<int> res;
    const auto& shapes = actor->getShapes();

    for(auto& shape : shapes)
    {
      ::physx::PxGeometryHolder px_geom = shape->getGeometry();
      if(px_geom.getType() == ::physx::PxGeometryType::eINVALID)
        continue;

      auto local_pose = shape->getLocalPose();
      ::physx::PxTransform world_pose = actor->getGlobalPose() * local_pose;

      ::physx::PxOverlapHit hit_buffer[256];
      ::physx::PxOverlapBuffer overlap_buffer(hit_buffer, 256);
      ::physx::PxQueryFilterData filter_data;
      filter_data.flags = ::physx::PxQueryFlag::eDYNAMIC | ::physx::PxQueryFlag::eSTATIC;

      bool has_hit = false;

      if(px_geom.getType() == ::physx::PxGeometryType::eTRIANGLEMESH)
      {
        ::physx::PxBounds3 aabb;
        ::physx::PxGeometryQuery::computeGeomBounds(aabb, px_geom.any(), world_pose);

        ::physx::PxBoxGeometry box_geom((aabb.maximum - aabb.minimum) * 0.5f);
        ::physx::PxTransform box_pose(aabb.getCenter());

        has_hit = ctx_->px_scene_->overlap(box_geom, box_pose, overlap_buffer, filter_data);
      }
      else
      {
        has_hit = ctx_->px_scene_->overlap(px_geom.any(), world_pose, overlap_buffer, filter_data);
      }

      if(has_hit)
      {
        for(::physx::PxU32 j = 0; j < overlap_buffer.getNbAnyHits(); ++j)
        {
          ::physx::PxActor* hit_actor = overlap_buffer.getAnyHit(j).actor;
          ::physx::PxRigidActor* rigid_actor = hit_actor->is<::physx::PxRigidActor>();

          if(rigid_actor != nullptr)
          {
            ActorData_t* data = static_cast<ActorData_t*>(rigid_actor->userData);
            if(data != nullptr)
            {
              if(data->body_id > 0)
                res.insert(data->body_id);
              else if(data->actor_id > 0)
                res.insert(data->actor_id);
            }
          }
        }
      }
    }

    return res;
  }

  void World::setGravity(const std::array<float, 3>& gravity)
  {
    ctx_->physx_mutex_.lock();
    ctx_->px_scene_->setGravity(::physx::PxVec3(
      static_cast<::physx::PxReal>(gravity[0]),
      static_cast<::physx::PxReal>(gravity[1]),
      static_cast<::physx::PxReal>(gravity[2])));
    ctx_->physx_mutex_.unlock();
  }

  void World::stepSimulation(const float delta)
  {
    ctx_->physx_mutex_.lock();
    if(ctx_->sub_step_ <= 1)
    {
      ctx_->px_scene_->simulate(delta != 0 ? delta : time_step_);
      ctx_->px_scene_->fetchResults(true);
    }
    else
    {
      for(size_t i = 0; i <  ctx_->sub_step_; i++)
      {
        if(i != 0)
        {
          for(auto& actor : actors_)
            actor.second->stepPose();
        }
        ctx_->px_scene_->simulate((delta != 0 ? delta : time_step_) / (float)ctx_->sub_step_);
        ctx_->px_scene_->fetchResults(true);
      }
    }
    ctx_->physx_mutex_.unlock();
  }

  void performRaycast(::physx::PxScene* scene,
                      const std::array<double, 3>& origin,
                      const std::array<double, 3>& destination,
                      float max_distance,
                      size_t ray_index,
                      std::vector<RaycastHitResult_t>& results)
  {
    ::physx::PxVec3 origin_vec((float)origin[0], (float)origin[1], (float)origin[2]);
    ::physx::PxVec3 destination_vec((float)destination[0], (float)destination[1], (float)destination[2]);

    ::physx::PxVec3 direction = destination_vec - origin_vec;
    ::physx::PxVec3 unit_dir = direction.getNormalized();

    ::physx::PxRaycastHit raycast_hits[2];
    ::physx::PxRaycastBuffer hit_buffer(raycast_hits, 2);

    bool hit = scene->raycast(origin_vec, unit_dir, max_distance, hit_buffer);

    RaycastHitResult_t result;
    result.hit = hit;

    if(hit)
    {
      result.distance = max_distance;
      size_t hit_count = hit_buffer.nbTouches;

      for(size_t i = 0; i < hit_count; i++)
      {
        if((hit_buffer.touches[i].distance < result.distance) &&
           (hit_buffer.touches[i].distance > 0.1))
        {
          const ::physx::PxRaycastHit& closest_hit = hit_buffer.touches[i];
          result.position = {closest_hit.position.x, closest_hit.position.y, closest_hit.position.z};
          result.normal = {closest_hit.normal.x, closest_hit.normal.y, closest_hit.normal.z};
          result.distance = closest_hit.distance;

          ActorData_t* data = static_cast<ActorData_t*>(closest_hit.actor->userData);
          if(data != nullptr)
          {
            result.actor_id = (closest_hit.actor != nullptr) ? data->actor_id : -1;
            result.body_id = (closest_hit.actor != nullptr) ? data->body_id : -1;
          }
          else
          {
            result.actor_id = -1;
            result.body_id = -1;
          }
        }
      }
    }

    results[ray_index] = result;
  }

  void World::performRaycastsInParallel(const std::vector<std::array<double, 3>>& origins,
                                        const std::vector<std::array<double, 3>>& destinations,
                                        float max_distance,
                                        std::vector<RaycastHitResult_t>& results)
  {
    size_t num_rays = origins.size();
    results.resize(num_rays);

    ctx_->physx_mutex_.lock();
    std::vector<std::thread> threads;
    for(size_t i = 0; i < num_rays; ++i)
      threads.emplace_back(performRaycast, ctx_->px_scene_.get(), origins[i], destinations[i], max_distance, i, std::ref(results));

    for(auto& thread : threads)
      thread.join();
    ctx_->physx_mutex_.unlock();
  }

} // namespace owds::physx