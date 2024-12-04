#include "overworld/Engine/Physics/PhysX/Actors/Actor.h"

#include <array>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>
#include <overworld/Utils/BitCast.h>

#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "overworld/Engine/Physics/PhysX/SharedContext.h"

namespace owds::physx {

  Actor::Actor(owds::physx::Context& ctx,
               const owds::Shape& collision_shape,
               const std::vector<owds::Shape>& visual_shapes) : owds::Actor(collision_shape, visual_shapes),
                                                                ctx_(ctx)
  {}

  void Actor::setStaticFriction(const float coefficient)
  {
    px_material_->setStaticFriction(coefficient);
  }

  void Actor::setDynamicFriction(const float coefficient)
  {
    px_material_->setDynamicFriction(coefficient);
  }

  void Actor::setRestitution(const float coefficient)
  {
    px_material_->setRestitution(coefficient);
  }

  std::array<float, 16> Actor::getModelMatrix() const
  {
    const auto px_pose = px_base_->getGlobalPose();
    const auto translation_mat = glm::translate(glm::mat4(1), glm::vec3(px_pose.p.x, px_pose.p.y, px_pose.p.z));
    const auto rotation_mat = glm::mat4_cast(glm::quat(px_pose.q.w, px_pose.q.x, px_pose.q.y, px_pose.q.z));

    return owds::BitCast<std::array<float, 16>>(translation_mat * rotation_mat);
  }

  std::pair<std::array<float, 3>, std::array<float, 3>> Actor::getPositionAndOrientation() const
  {
    const auto px_pose = px_base_->getGlobalPose();
    const auto position = glm::vec3(px_pose.p.x, px_pose.p.y, px_pose.p.z);
    const auto euler_angles = glm::eulerAngles(glm::quat(px_pose.q.w, px_pose.q.x, px_pose.q.y, px_pose.q.z));

    return {
      owds::BitCast<std::array<float, 3>>(position),
      owds::BitCast<std::array<float, 3>>(euler_angles)};
  }

  void Actor::setupPhysicsShape(const owds::ShapeBox& shape)
  {
    px_geometries_.emplace_back(std::make_unique<::physx::PxBoxGeometry>(
      static_cast<::physx::PxReal>(shape.half_extents_[0]),
      static_cast<::physx::PxReal>(shape.half_extents_[1]),
      static_cast<::physx::PxReal>(shape.half_extents_[2])));
  }

  void Actor::setupPhysicsShape(const owds::ShapeCapsule& shape)
  {
    px_geometries_.emplace_back(std::make_unique<::physx::PxCapsuleGeometry>(
      static_cast<::physx::PxReal>(shape.radius_),
      static_cast<::physx::PxReal>(shape.height_) / 2));
  }

  static auto createPxCookingParams(bool minimize_memory_footprint)
  {
    auto params = ::physx::PxCookingParams(::physx::PxTolerancesScale());
    params.convexMeshCookingType = ::physx::PxConvexMeshCookingType::eQUICKHULL;
    params.midphaseDesc = ::physx::PxMeshMidPhase::eBVH34;
    params.suppressTriangleMeshRemapTable = minimize_memory_footprint;
    params.buildGPUData = !minimize_memory_footprint;
    params.meshPreprocessParams &= ~static_cast<::physx::PxMeshPreprocessingFlags>(
      ::physx::PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
    params.meshPreprocessParams &= ~static_cast<::physx::PxMeshPreprocessingFlags>(
      ::physx::PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
    params.meshPreprocessParams |=
      ::physx::PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES;
    params.meshPreprocessParams |=
      ::physx::PxMeshPreprocessingFlag::eENABLE_INERTIA;

    // Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
    // and worse cooking performance. Cooking time is better when more triangles per leaf are used.
    params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = 15;
    params.midphaseDesc.mBVH34Desc.buildStrategy = ::physx::PxBVH34BuildStrategy::eFAST;
    params.midphaseDesc.mBVH34Desc.quantized = minimize_memory_footprint;
    return params;
  }

  static auto createPxTriangleMesh(const ::physx::PxCookingParams& cooking_params,
                                   const std::vector<::physx::PxVec3>& vertices,
                                   const std::vector<std::uint32_t>& indices)
  {
    auto sdf_desc = ::physx::PxSDFDesc();
    sdf_desc.spacing = 0.05f;
    sdf_desc.subgridSize = 6;
    sdf_desc.bitsPerSubgridPixel = ::physx::PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL;
    sdf_desc.numThreadsForSdfConstruction = 16;
    assert(sdf_desc.isValid());

    auto mesh_desc = ::physx::PxTriangleMeshDesc();
    mesh_desc.points.count = vertices.size();
    mesh_desc.points.stride = sizeof(vertices[0]);
    mesh_desc.points.data = vertices.data();
    mesh_desc.triangles.count = indices.size() / 3;
    mesh_desc.triangles.stride = sizeof(indices[0]) * 3;
    mesh_desc.triangles.data = indices.data();
    mesh_desc.sdfDesc = &sdf_desc;
    assert(mesh_desc.isValid());

    // assert(PxValidateConvexMesh(params, mesh_desc));

    ::physx::PxDefaultMemoryOutputStream writeBuffer;
    ::physx::PxTriangleMeshCookingResult::Enum result;

    assert(PxCookTriangleMesh(cooking_params, mesh_desc, writeBuffer, &result));

    using ResultTy = decltype(result);

    switch(result)
    {
    case ResultTy::eSUCCESS:
      break;
    case ResultTy::eLARGE_TRIANGLE:
      assert(false && "eLARGE_TRIANGLE");
      break;
    case ResultTy::eEMPTY_MESH:
      assert(false && "eEMPTY_MESH");
      break;
    case ResultTy::eFAILURE:
      assert(false && "eFAILURE");
      break;
    }

    ::physx::PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());

    const auto& sdk = owds::physx::Context::createContext()->px_physics_;
    const auto px_mesh = sdk->createTriangleMesh(readBuffer);

    assert(px_mesh);

    return px_mesh;
  }

  void Actor::setupPhysicsShape(const owds::ShapeCustomMesh& shape)
  {
    const auto& s_ctx = owds::physx::Context::createContext();

    const auto params = createPxCookingParams(ctx_.minimize_memory_footprint_);

    for(const auto& mesh : shape.custom_model_.get().meshes_)
    {
      if(!s_ctx->px_cached_meshes.count(mesh.id_))
      {
        if(mesh.vertices_.size() < 4)
        {
          printf("[%s:%s] Ignoring! Less than 4 vertices..\n",
                 shape.custom_model_.get().source_path_.c_str(),
                 mesh.name_.c_str());
          continue;
        }

        std::vector<::physx::PxVec3> vertices;
        vertices.reserve(mesh.vertices_.size());

        for(const auto& vertex : mesh.vertices_)
        {
          vertices.emplace_back(
            vertex.position_.x,
            vertex.position_.y,
            vertex.position_.z);
        }

        s_ctx->px_cached_meshes[mesh.id_] = createPxTriangleMesh(params, vertices, mesh.indices_);
      }

      px_geometries_.emplace_back(std::make_unique<::physx::PxTriangleMeshGeometry>(
        s_ctx->px_cached_meshes[mesh.id_].get(),
        ::physx::PxMeshScale(::physx::PxVec3(
          static_cast<::physx::PxReal>(shape.scale_.x),
          static_cast<::physx::PxReal>(shape.scale_.y),
          static_cast<::physx::PxReal>(shape.scale_.z)))));
    }
  }

  void Actor::setupPhysicsShape(const owds::ShapeCylinder& shape)
  {
    setupPhysicsShape(owds::ShapeCustomMesh{
      glm::vec3{shape.radius_, shape.height_, shape.radius_},
      {},
      shape.cylinder_model_
    });
  }

  void Actor::setupPhysicsShape(const owds::ShapeDummy& shape)
  {
    (void)shape;
  }

  void Actor::setupPhysicsShape(const owds::ShapeSphere& shape)
  {
    px_geometries_.emplace_back(std::make_unique<::physx::PxSphereGeometry>(
      static_cast<::physx::PxReal>(shape.radius_)));
  }

} // namespace owds::physx