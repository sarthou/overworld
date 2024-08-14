#include "overworld/Engine/Graphics/Assimp/ModelLoader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <filesystem>
#include <fstream>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <iostream>

#include "overworld/Engine/Common/Models/Loaders/ColladaLoader.h"
#include "overworld/Engine/Common/Models/Loaders/ObjLoader.h"
#include "overworld/Engine/Common/Models/Loaders/StlLoader.h"
#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Common/Models/Model.h"

namespace owds::assimp {

  owds::Mesh processMesh(const aiMesh* mesh)
  {
    owds::Mesh out_mesh = owds::Mesh::create();
    out_mesh.name_ = mesh->mName.C_Str();

    for(auto i = 0u; i < mesh->mNumVertices; i++)
    {
      Vertex vertex{};

      vertex.position_ = {
        mesh->mVertices[i].x,
        mesh->mVertices[i].y,
        mesh->mVertices[i].z};

      vertex.normal_ = {
        mesh->mNormals[i].x,
        mesh->mNormals[i].y,
        mesh->mNormals[i].z};

      if(mesh->mTextureCoords[0])
      {
        vertex.uv_ = {
          mesh->mTextureCoords[0][i].x,
          mesh->mTextureCoords[0][i].y};
      }
      else
        vertex.uv_ = {0, 0};

      out_mesh.vertices_.emplace_back(vertex);
    }

    for(auto i = 0u; i < mesh->mNumFaces; i++)
    {
      const aiFace face = mesh->mFaces[i];

      for(auto j = 0u; j < face.mNumIndices; j++)
      {
        out_mesh.indices_.emplace_back(face.mIndices[j]);
      }
    }

    return out_mesh;
  }

  void processNode(owds::Model& out, const aiNode* node, const aiScene* scene)
  {
    for(auto i = 0u; i < node->mNumMeshes; i++)
    {
      out.meshes_.emplace_back(processMesh(scene->mMeshes[node->mMeshes[i]]));
    }

    for(auto i = 0u; i < node->mNumChildren; i++)
    {
      processNode(out, node->mChildren[i], scene);
    }
  }

  bool loadModel(owds::Model& out, const std::filesystem::path& path)
  {
    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile(
      path,
      0 |
        aiProcess_Triangulate |
        aiProcess_SortByPType |
        aiProcess_GenNormals |
        aiProcess_FlipUVs |
        aiProcess_GenUVCoords |
        aiProcess_GlobalScale);

    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
      return false;
    }

    out.source_path_ = path.string();
    processNode(out, scene->mRootNode, scene);

    return true;
  }

  std::unique_ptr<owds::Model> ModelLoader::load(const std::filesystem::path& path) const
  {
    if(path.string().rfind(".stl") != std::string::npos)
      return StlLoader::read(path.string());
    else if(path.string().rfind(".obj") != std::string::npos)
    {
      ObjLoader loader;
      return loader.read(path.string());
    }
    else if(path.string().rfind(".dae") != std::string::npos)
    {
      ColladaLoader loader;
      return loader.read(path.string());
    }

    auto model = std::make_unique<owds::Model>(owds::Model::create());
    model->source_path_ = path.string();

    if(!loadModel(*model, path))
    {
      return nullptr;
    }

    return model;
  }

  void ModelLoader::computeTangentSpace(std::unique_ptr<owds::Model> model)
  {
    for(auto& mesh : model->meshes_)
      computeTangentSpace(mesh);
  }

  void ModelLoader::computeTangentSpace(Mesh& mesh)
  {
    for(unsigned int i = 0; i < mesh.indices_.size(); i = i + 3)
    {
      glm::vec3& vertex0 = mesh.vertices_.at(mesh.indices_.at(i)).position_;
      glm::vec3& vertex1 = mesh.vertices_.at(mesh.indices_.at(i + 1)).position_;
      glm::vec3& vertex2 = mesh.vertices_.at(mesh.indices_.at(i + 3)).position_;

      glm::vec3 normal = glm::cross((vertex1 - vertex0), (vertex2 - vertex0));

      glm::vec3 delta_pos;
      if(vertex0 == vertex1)
        delta_pos = vertex2 - vertex0;
      else
        delta_pos = vertex1 - vertex0;

      glm::vec2& uv0 = mesh.vertices_.at(mesh.indices_.at(i)).uv_;
      glm::vec2& uv1 = mesh.vertices_.at(mesh.indices_.at(i + 1)).uv_;
      // lm::vec2& uv2 = mesh.vertices_.at(mesh.indices_.at(i + 2)).uv_;

      glm::vec2 delta_uv1 = uv1 - uv0;

      glm::vec3 tan;
      // avoid divion with 0
      if(delta_uv1.s != 0)
        tan = delta_pos / delta_uv1.s;
      else
        tan = delta_pos / 1.0f;

      tan = glm::normalize(tan - glm::dot(normal, tan) * normal);

      glm::vec3 bin = glm::normalize(glm::cross(tan, normal));

      // write into array - for each vertex of the face the same value
      mesh.vertices_[mesh.indices_.at(i)].tangent_ = tan;
      mesh.vertices_[mesh.indices_.at(i + 1)].tangent_ = tan;
      mesh.vertices_[mesh.indices_.at(i + 2)].tangent_ = tan;

      mesh.vertices_[mesh.indices_.at(i)].bitangent_ = bin;
      mesh.vertices_[mesh.indices_.at(i + 1)].bitangent_ = bin;
      mesh.vertices_[mesh.indices_.at(i + 2)].bitangent_ = bin;
    }
  }
} // namespace owds::assimp