#include "overworld/Engine/Common/Models/Loaders/ObjLoader.h"

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>
#include <memory>
#include <string>
#include <vector>

#include "overworld/Engine/Common/Models/Material.h"
#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Models/Vertex.h"
#include "overworld/Utility/ShellDisplay.h"

namespace owds {

  std::unique_ptr<owds::Model> ObjLoader::read(const std::string& path)
  {
    std::string mtl_path;
    size_t pos = path.find_last_of("/\\");
    if(pos != std::string::npos)
      mtl_path = path.substr(0, pos);

    parseFromFile(path, mtl_path);

    if(isValid() == false)
      return nullptr;

    tinyobj::Attrib_t attributes = getAttrib();
    std::vector<tinyobj::Mesh_t> meshes = getMeshes();
    std::vector<tinyobj::Material_t> materials = getMaterials();

    std::unique_ptr<owds::Model> model = getModel(attributes, meshes, materials);
    if(model != nullptr)
      model->source_path_ = path;

    return model;
  }

  Vertex ObjLoader::getVertex(const tinyobj::Attrib_t& attribute,
                              const tinyobj::Index_t& indexes)
  {
    Vertex vtx;
    vtx.position_.x = attribute.vertices[3 * indexes.vertex_index];
    vtx.position_.y = attribute.vertices[3 * indexes.vertex_index + 1];
    vtx.position_.z = attribute.vertices[3 * indexes.vertex_index + 2];

    if(attribute.texcoords.size())
    {
      int uv0_index = 2 * indexes.texcoord_index;
      int uv1_index = 2 * indexes.texcoord_index + 1;
      if(uv0_index >= 0 && uv1_index >= 0 &&
         (uv1_index < int(attribute.texcoords.size())))
      {
        vtx.uv_.x = attribute.texcoords[uv0_index];
        vtx.uv_.y = attribute.texcoords[uv1_index];
      }
      else
      {
        vtx.uv_.x = 0;
        vtx.uv_.y = 0;
      }
    }
    else
    {
      vtx.uv_.x = 0.5;
      vtx.uv_.y = 0.5;
    }

    return vtx;
  }

  std::unique_ptr<owds::Model> ObjLoader::getModel(const tinyobj::Attrib_t& attribute,
                                                   std::vector<tinyobj::Mesh_t>& meshes,
                                                   std::vector<tinyobj::Material_t>& materials)
  {
    (void)materials; // TODO
    auto model = std::make_unique<Model>(Model::create());

    for(const auto& tiny_mesh : meshes)
    {
      model->meshes_.emplace_back(Mesh::create());
      Mesh* owds_mesh = &(model->meshes_.back());
      owds_mesh->name_ = tiny_mesh.name;

      int face_count = tiny_mesh.indices.size();

      for(int f = 0; f < face_count; f += 3)
      {
        int vtx_base_index = owds_mesh->vertices_.size();

        tinyobj::Index_t v_0 = tiny_mesh.indices[f];
        Vertex vtx0 = getVertex(attribute, v_0);

        tinyobj::Index_t v_1 = tiny_mesh.indices[f + 1];
        Vertex vtx1 = getVertex(attribute, v_1);

        tinyobj::Index_t v_2 = tiny_mesh.indices[f + 2];
        Vertex vtx2 = getVertex(attribute, v_2);

        unsigned n0_index = tiny_mesh.indices[f].normal_index;
        unsigned n1_index = tiny_mesh.indices[f + 1].normal_index;
        unsigned n2_index = tiny_mesh.indices[f + 2].normal_index;

        bool has_normals = (attribute.normals.empty() == false);
        if(has_normals)
        {
          unsigned int max_index = 0;

          max_index = std::max(max_index, 3 * n0_index + 0);
          max_index = std::max(max_index, 3 * n0_index + 1);
          max_index = std::max(max_index, 3 * n0_index + 2);
          max_index = std::max(max_index, 3 * n1_index + 0);
          max_index = std::max(max_index, 3 * n1_index + 1);
          max_index = std::max(max_index, 3 * n1_index + 2);
          max_index = std::max(max_index, 3 * n2_index + 0);
          max_index = std::max(max_index, 3 * n2_index + 1);
          max_index = std::max(max_index, 3 * n2_index + 2);

          has_normals = (max_index < attribute.normals.size());
        }

        if(has_normals == false)
        {
          auto normal = glm::cross((vtx1.position_ - vtx0.position_), (vtx2.position_ - vtx0.position_));
          float len = glm::length(normal);
          // skip degenerate triangles
          if(len > FLT_EPSILON)
            normal = glm::normalize(normal);
          else
            normal = glm::vec3(0.f, 0.f, 0.f);

          vtx0.normal_ = normal;
          vtx1.normal_ = normal;
          vtx2.normal_ = normal;
        }
        else
        {
          vtx0.normal_.x = attribute.normals[3 * n0_index + 0];
          vtx0.normal_.y = attribute.normals[3 * n0_index + 1];
          vtx0.normal_.z = attribute.normals[3 * n0_index + 2];
          vtx1.normal_.x = attribute.normals[3 * n1_index + 0];
          vtx1.normal_.y = attribute.normals[3 * n1_index + 1];
          vtx1.normal_.z = attribute.normals[3 * n1_index + 2];
          vtx2.normal_.x = attribute.normals[3 * n2_index + 0];
          vtx2.normal_.y = attribute.normals[3 * n2_index + 1];
          vtx2.normal_.z = attribute.normals[3 * n2_index + 2];
        }

        owds_mesh->vertices_.push_back(vtx0);
        owds_mesh->vertices_.push_back(vtx1);
        owds_mesh->vertices_.push_back(vtx2);
        owds_mesh->indices_.push_back(vtx_base_index);
        owds_mesh->indices_.push_back(vtx_base_index + 1);
        owds_mesh->indices_.push_back(vtx_base_index + 2);
      }
    }

    return model;
  }

} // namespace owds