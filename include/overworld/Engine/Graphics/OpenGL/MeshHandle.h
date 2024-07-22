#ifndef OWDS_GRAPHICS_OPENGL_MESHHANDLE_H
#define OWDS_GRAPHICS_OPENGL_MESHHANDLE_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include <vector>

#include "glad/glad.h"
#include "overworld/Engine/Common/Models/Color.h"
#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Graphics/OpenGL/Shader.h"
#include "overworld/Engine/Graphics/OpenGL/Texture2D.h"

namespace owds {

  class MeshHandle
  {
  public:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture2D> textures;
    float shininess;
    float specular;
    Color color;
    unsigned int vao; // vertex array object

    MeshHandle(const Mesh& mesh, const std::vector<Texture2D>& textures)
    {
      this->vertices = mesh.vertices_;
      this->indices = mesh.indices_;
      this->textures = textures;

      setupMesh();
    }

    void draw(Shader& shader) const
    {
      unsigned int diffuse_nr = 1;
      unsigned int specular_nr = 1;
      unsigned int normal_nr = 1;
      unsigned int height_nr = 1;
      for(unsigned int i = 0; i < textures.size(); i++)
      {
        glActiveTexture(GL_TEXTURE0 + i); // active proper texture unit before binding
        std::string number;
        std::string name;
        if(textures[i].type_ == texture_diffuse)
        {
          name = "texture_diffuse";
          number = std::to_string(diffuse_nr++);
        }
        else if(textures[i].type_ == texture_specular)
        {
          name = "texture_specular";
          number = std::to_string(specular_nr++);
        }
        else if(textures[i].type_ == texture_normal)
        {
          name = "texture_normal";
          number = std::to_string(normal_nr++);
        }
        else if(textures[i].type_ == texture_height)
        {
          name = "texture_height";
          number = std::to_string(height_nr++);
        }

        // now set the sampler to the correct texture unit
        glUniform1i(glGetUniformLocation(shader.id_, (name + number).c_str()), i);
        // and finally bind the texture
        glBindTexture(GL_TEXTURE_2D, textures[i].id_);
      }

      shader.setFloat("material.shininess", shininess);
      shader.setFloat("material.specular", (specular_nr == 1) ? specular : -1);
      shader.setVec4("material.color", glm::vec4(color.r_, color.g_, color.b_, (diffuse_nr == 1) ? color.a_ : 0));

      glBindVertexArray(vao);
      glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
      glBindVertexArray(0);

      // always good practice to set everything back to defaults once configured.
      glActiveTexture(GL_TEXTURE0);
    }

  private:
    unsigned int vbo; // vertex buffer object
    unsigned int ebo; //

    // initializes all the buffer objects/arrays
    void setupMesh()
    {
      // create buffers/arrays
      glGenVertexArrays(1, &vao);
      glGenBuffers(1, &vbo);
      glGenBuffers(1, &ebo);

      glBindVertexArray(vao);
      // load data into vertex buffers
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
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
      // glEnableVertexAttribArray(3);
      // glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Tangent));
      //// vertex bitangent
      // glEnableVertexAttribArray(4);
      // glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Bitangent));
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
