#include "overworld/Engine/Physics/PhysX/Actors/Actor.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <common/PxTolerancesScale.h>
#include <cooking/PxBVH34MidphaseDesc.h>
#include <cooking/PxSDFDesc.h>
#include <cooking/PxTriangleMeshDesc.h>
#include <cstddef>
#include <extensions/PxDefaultStreams.h>
#include <filesystem>
#include <fstream>
#include <glm/detail/type_quat.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>
#include <string>
#include <sys/stat.h>

#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Shapes/ShapeBox.h"
#include "overworld/Engine/Common/Shapes/ShapeCapsule.h"
#include "overworld/Engine/Common/Shapes/ShapeCustomMesh.h"
#include "overworld/Engine/Common/Shapes/ShapeCylinder.h"
#include "overworld/Engine/Common/Shapes/ShapeDummy.h"
#include "overworld/Engine/Common/Shapes/ShapeSphere.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/World.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "overworld/Engine/Physics/PhysX/SharedContext.h"
#include "overworld/Utils/BitCast.h"

using namespace std::chrono;

namespace owds::physx {

  bool Actor::use_cache_input = false;
  bool Actor::use_cache_output = false;

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

  owds::AABB_t Actor::getAABB()
  {
    ::physx::PxBounds3 px_aabb = px_base_->getWorldBounds();
    AABB_t aabb({px_aabb.minimum.x, px_aabb.minimum.y, px_aabb.minimum.z},
                {px_aabb.maximum.x, px_aabb.maximum.y, px_aabb.maximum.z});
    return aabb;
  }

  owds::AABB_t Actor::getLocalAABB()
  {
    ::physx::PxBounds3 px_aabb;
    for(auto& shape : px_shapes_)
    {
      ::physx::PxGeometryHolder px_geom = shape->getGeometry();
      auto local_pose = shape->getLocalPose();
      ::physx::PxBounds3 geom_bounds;
      ::physx::PxGeometryQuery::computeGeomBounds(geom_bounds, px_geom.any(), ::physx::PxTransform(::physx::PxIdentity));
      ::physx::PxBounds3 shape_local_bounds = ::physx::PxBounds3::transformSafe(local_pose, geom_bounds);
      if(px_aabb.isValid())
        px_aabb.include(shape_local_bounds);
      else
        px_aabb = shape_local_bounds;
    }

    AABB_t aabb({px_aabb.minimum.x, px_aabb.minimum.y, px_aabb.minimum.z},
                {px_aabb.maximum.x, px_aabb.maximum.y, px_aabb.maximum.z});
    return aabb;
  }

  std::array<float, 16> Actor::getModelMatrix() const
  {
    const auto px_pose = px_base_->getGlobalPose();
    const auto translation_mat = glm::translate(glm::mat4(1), glm::vec3(px_pose.p.x, px_pose.p.y, px_pose.p.z));
    const auto rotation_mat = glm::mat4_cast(glm::quat(px_pose.q.w, px_pose.q.x, px_pose.q.y, px_pose.q.z));

    return owds::BitCast<std::array<float, 16>>(translation_mat * rotation_mat);
  }

  std::pair<std::array<double, 3>, std::array<double, 4>> Actor::getPositionAndOrientation() const
  {
    const auto px_pose = px_base_->getGlobalPose();
    return {
      {(double)px_pose.p.x, (double)px_pose.p.y, (double)px_pose.p.z},
      {(double)px_pose.q.x, (double)px_pose.q.y, (double)px_pose.q.z, (double)px_pose.q.w}
    };
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

  static void cookTriangleMesh(const ::physx::PxCookingParams& cooking_params,
                               const std::vector<::physx::PxVec3>& vertices,
                               const std::vector<std::uint32_t>& indices,
                               ::physx::PxDefaultMemoryOutputStream& buffer)
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

    ::physx::PxTriangleMeshCookingResult::Enum result;

    assert(PxCookTriangleMesh(cooking_params, mesh_desc, buffer, &result));

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
  }

  static ::physx::PxTriangleMesh* loadCookedFromDisk(const std::string& path)
  {
    struct stat stat_buffer;
    if(stat(path.c_str(), &stat_buffer) == 0)
    {
      std::ifstream in_file(path, std::ios::binary | std::ios::ate);
      if(!in_file.is_open())
      {
        std::cerr << "Failed to open file for reading: " << path << std::endl;
        return nullptr;
      }

      std::streamsize size = in_file.tellg();
      in_file.seekg(0, std::ios::beg);

      std::vector<char> buffer(size);
      if(!in_file.read(buffer.data(), size))
      {
        std::cerr << "Failed to read data from file: " << path << std::endl;
        return nullptr;
      }

      ::physx::PxDefaultMemoryInputData read_buffer(reinterpret_cast<::physx::PxU8*>(buffer.data()), static_cast<::physx::PxU32>(size));
      const auto& sdk = owds::physx::Context::createContext()->px_physics_;
      ::physx::PxTriangleMesh* triangle_mesh = sdk->createTriangleMesh(read_buffer);
      if(triangle_mesh == nullptr)
      {
        std::cerr << "Failed to create triangle mesh from cooked data!" << std::endl;
        return nullptr;
      }

      if(Actor::use_cache_input == false)
      {
        Actor::use_cache_input = true;
        std::cout << "Cooked mesh successfully loaded from /tmp/overworld " << std::endl;
      }
      return triangle_mesh;
    }
    else
      return nullptr;
  }

  static bool saveCookedData(const ::physx::PxDefaultMemoryOutputStream& buffer, const std::string& path)
  {
    std::string directory = path.substr(0, path.find_last_of('/'));
    std::filesystem::create_directories(directory);

    std::ofstream out_file(path, std::ios::binary);
    if(!out_file.is_open())
    {
      std::cerr << "Failed to open file for writing: " << path << std::endl;
      return false;
    }
    out_file.write((const char*)buffer.getData(), buffer.getSize());
    out_file.close();

    if(Actor::use_cache_output == false)
    {
      Actor::use_cache_output = true;
      std::cout << "Cooked mesh saved to /tmp/overworld" << std::endl;
    }
    return true;
  }

  static auto createPxTriangleMesh(const ::physx::PxCookingParams& cooking_params,
                                   const std::vector<::physx::PxVec3>& vertices,
                                   const std::vector<std::uint32_t>& indices,
                                   const std::string& path)
  {
    ::physx::PxDefaultMemoryOutputStream write_buffer;
    cookTriangleMesh(cooking_params, vertices, indices, write_buffer);

    saveCookedData(write_buffer, path);

    ::physx::PxDefaultMemoryInputData read_buffer(write_buffer.getData(), write_buffer.getSize());

    const auto& sdk = owds::physx::Context::createContext()->px_physics_;
    const auto px_mesh = sdk->createTriangleMesh(read_buffer);

    assert(px_mesh);

    return px_mesh;
  }

  static ::physx::PxTriangleMesh* createPxTriangleMesh(const owds::Mesh& mesh,
                                                       const ::physx::PxCookingParams& params,
                                                       const std::string& path)
  {
    std::vector<::physx::PxVec3> vertices;
    vertices.reserve(mesh.vertices_.size());

    for(const auto& vertex : mesh.vertices_)
    {
      vertices.emplace_back(
        vertex.position_.x,
        vertex.position_.y,
        vertex.position_.z);
    }

    return createPxTriangleMesh(params, vertices, mesh.indices_, path);
  }

  static std::string getModelName(const std::string& path)
  {
    std::string res;
    size_t pose = path.find_last_of('/');
    if(pose == std::string::npos)
      res = path;
    else
      res = path.substr(pose + 1);

    std::replace(res.begin(), res.end(), '.', '_');

    return res;
  }

  void Actor::setupPhysicsShape(const owds::ShapeCustomMesh& shape)
  {
    const auto params = createPxCookingParams(ctx_.minimize_memory_footprint_);

    std::string model_name = getModelName(shape.custom_model_.get().source_path_);

    for(const auto& mesh : shape.custom_model_.get().meshes_)
    {
      std::string bin_path = "/tmp/overworld/" + model_name;
      if(mesh.name_.empty() == false)
        bin_path += "_" + mesh.name_ + ".bin";

      if(mesh.vertices_.size() < 4)
      {
        printf("[%s:%s] Ignoring! Less than 4 vertices..\n",
               shape.custom_model_.get().source_path_.c_str(),
               mesh.name_.c_str());
        continue;
      }

      auto* tringle_mesh = loadCookedFromDisk(bin_path);
      if(tringle_mesh == nullptr)
        tringle_mesh = createPxTriangleMesh(mesh, params, bin_path);

      px_geometries_.emplace_back(std::make_unique<::physx::PxTriangleMeshGeometry>(
        tringle_mesh,
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