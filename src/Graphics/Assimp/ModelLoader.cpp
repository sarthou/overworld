#include "overworld/Graphics/Assimp/ModelLoader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <filesystem>
#include <fstream>

#include "overworld/Graphics/Base/Model.h"

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
          1.f - mesh->mTextureCoords[0][i].y}; // flip
      }

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
        aiProcess_GenNormals);

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
    auto model = std::make_unique<owds::Model>(owds::Model::create());
    model->source_path_ = path.string();

    if(!loadModel(*model, path))
    {
      return nullptr;
    }

    return model;
  }
} // namespace owds::assimp