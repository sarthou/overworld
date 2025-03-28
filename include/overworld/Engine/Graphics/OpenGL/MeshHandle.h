#ifndef OWDS_GRAPHICS_OPENGL_MESHHANDLE_H
#define OWDS_GRAPHICS_OPENGL_MESHHANDLE_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "glad/glad.h"
#include "overworld/Engine/Common/Models/Color.h"
#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Graphics/OpenGL/Shader.h"
#include "overworld/Engine/Graphics/OpenGL/Texture2D.h"

namespace owds {

  struct MeshMaterialHandle_t
  {
    std::vector<Texture2D> textures;
    float shininess;
    float specular;
    Color color;
  };

  class MeshHandle
  {
  public:
    uint32_t id;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::unordered_map<uint32_t, MeshMaterialHandle_t> materials;

    MeshHandle(const Mesh& mesh)
    {
      this->vertices = mesh.vertices_;
      this->indices = mesh.indices_;

      setupMesh();
    }

    void drawId(const Shader& shader, uint32_t model_id) const
    {
      // model_id *= 4'000'000;
      shader.setVec4("color", glm::vec4((uint8_t)(model_id >> 24) / 255.,
                                        (uint8_t)(model_id >> 16) / 255.,
                                        (uint8_t)(model_id >> 8) / 255.,
                                        (uint8_t)(model_id) / 255.));

      glBindVertexArray(vao_);
      glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
      glBindVertexArray(0);
    }

    void draw(const Shader& shader, uint32_t model_id, unsigned int texture_pose_offset = 0) const
    {
      auto material_it = materials.find(model_id);
      if(material_it == materials.end())
        return;

      unsigned int diffuse_nr = 1;
      unsigned int specular_nr = 1;
      unsigned int normal_nr = 1;
      unsigned int height_nr = 1;
      unsigned int nb_used = 0;
      for(unsigned int i = 0; i < material_it->second.textures.size(); i++)
      {
        glActiveTexture(GL_TEXTURE0 + texture_pose_offset + i); // active proper texture unit before binding
        std::string number;
        std::string name;
        if(material_it->second.textures[i].type_ == texture_diffuse)
        {
          name = "material.texture_diffuse";
          number = std::to_string(diffuse_nr++);
        }
        else if(material_it->second.textures[i].type_ == texture_specular)
        {
          name = "material.texture_specular";
          number = std::to_string(specular_nr++);
        }
        else if(material_it->second.textures[i].type_ == texture_normal)
        {
          name = "material.texture_normal";
          number = std::to_string(normal_nr++);
          shader.setFloat("material.use_normal", 1.);
        }
        else if(material_it->second.textures[i].type_ == texture_height)
        {
          name = "material.texture_height";
          number = std::to_string(height_nr++);
        }

        // now bind the texture
        glBindTexture(GL_TEXTURE_2D, material_it->second.textures[i].id_);
        // and finally set the sampler to the correct texture unit
        shader.setInt(name + number, texture_pose_offset + i);
        nb_used++;
      }

      if(diffuse_nr == 1)
      {
        glActiveTexture(GL_TEXTURE0 + texture_pose_offset + nb_used);
        glBindTexture(GL_TEXTURE_2D, 0);
        shader.setInt("material.texture_diffuse1", texture_pose_offset + nb_used);
        nb_used++;
      }

      if(specular_nr == 1)
      {
        glActiveTexture(GL_TEXTURE0 + texture_pose_offset + nb_used);
        glBindTexture(GL_TEXTURE_2D, 0);
        shader.setInt("material.texture_specular1", texture_pose_offset + nb_used);
        nb_used++;
      }

      if(normal_nr == 1)
      {
        glActiveTexture(GL_TEXTURE0 + texture_pose_offset + nb_used);
        glBindTexture(GL_TEXTURE_2D, 0);
        shader.setInt("material.texture_normal1", texture_pose_offset + nb_used);
        shader.setFloat("material.use_normal", 0.);
        nb_used++;
      }

      if(height_nr == 1)
      {
        glActiveTexture(GL_TEXTURE0 + texture_pose_offset + nb_used);
        glBindTexture(GL_TEXTURE_2D, 0);
        shader.setInt("material.texture_height1", texture_pose_offset + nb_used);
        nb_used++;
      }

      shader.setFloat("material.shininess", material_it->second.shininess);
      shader.setFloat("material.specular", (specular_nr == 1) ? material_it->second.specular : -1);
      shader.setVec4("material.color", glm::vec4(material_it->second.color.r_,
                                                 material_it->second.color.g_,
                                                 material_it->second.color.b_,
                                                 (diffuse_nr == 1) ? material_it->second.color.a_ : 0));

      glBindVertexArray(vao_);
      glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
      glBindVertexArray(0);

      // always good practice to set everything back to defaults once configured.
      glActiveTexture(GL_TEXTURE0);
    }

  private:
    unsigned int vbo_; // vertex buffer object
    unsigned int ebo_; //
    unsigned int vao_; // vertex array object

    // initializes all the buffer objects/arrays
    void setupMesh()
    {
      // create buffers/arrays
      glGenVertexArrays(1, &vao_);
      glGenBuffers(1, &vbo_);
      glGenBuffers(1, &ebo_);

      glBindVertexArray(vao_);
      // load data into vertex buffers
      glBindBuffer(GL_ARRAY_BUFFER, vbo_);
      glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

      // set the vertex attribute pointers
      // vertex Positions
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
      // vertex normals
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal_));
      // vertex texture coords
      glEnableVertexAttribArray(2);
      glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv_));
      // vertex tangent
      glEnableVertexAttribArray(3);
      glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tangent_));
      // vertex bitangent
      glEnableVertexAttribArray(4);
      glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, bitangent_));
      //// ids
      // glEnableVertexAttribArray(5);
      // glVertexAttribIPointer(5, 4, GL_INT, sizeof(Vertex), (void*)offsetof(Vertex, m_BoneIDs));

      // weights
      // glEnableVertexAttribArray(6);
      // glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, m_Weights));
      // glBindVertexArray(0);
    }
  };

} // namespace owds

#endif // OWDS_GRAPHICS_OPENGL_MESHHANDLE_H
