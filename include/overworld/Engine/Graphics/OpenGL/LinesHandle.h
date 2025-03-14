#ifndef OWDS_GRAPHICS_OPENGL_LINESHANDLE_H
#define OWDS_GRAPHICS_OPENGL_LINESHANDLE_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "glad/glad.h"
#include "overworld/Engine/Common/Debug/DebugLine.h"
#include "overworld/Engine/Common/Models/Color.h"
#include "overworld/Engine/Graphics/OpenGL/Shader.h"

namespace owds {
  class Actor;

  class LinesHandle
  {
  public:
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;
    glm::vec3 color;
    Actor* actor;

    LinesHandle(const DebugLine& lines) : vertices(lines.vertices_),
                                          indices(lines.indices_),
                                          color(lines.color_),
                                          actor(lines.linked_actor_),
                                          enabled_(true)
    {
      setupBuffers();
    }

    LinesHandle(const std::vector<glm::vec3>& vertices,
                const std::vector<unsigned int>& indices,
                const glm::vec3& color) : actor(nullptr), enabled_(true)
    {
      this->vertices = vertices;
      this->indices = indices;
      this->color = color;

      setupBuffers();
    }

    ~LinesHandle()
    {
      glDeleteVertexArrays(1, &vao_);
      glDeleteBuffers(1, &vbo_);
      glDeleteBuffers(1, &ebo_);
    }

    LinesHandle(const LinesHandle& other) = delete;

    void disable()
    {
      enabled_ = false;
    }

    bool isEnabled() { return enabled_; }

    void draw(const Shader& shader) const
    {
      shader.setVec4("color", glm::vec4(color, 1.0));

      glBindVertexArray(vao_);
      glDrawElements(GL_LINES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
      glBindVertexArray(0);
    }

  private:
    unsigned int vbo_; // vertex buffer object
    unsigned int ebo_; //
    unsigned int vao_; // vertex array object

    bool enabled_;

    // initializes all the buffer objects/arrays
    void setupBuffers()
    {
      // create buffers/arrays
      glGenVertexArrays(1, &vao_);
      glGenBuffers(1, &vbo_);
      glGenBuffers(1, &ebo_);

      glBindVertexArray(vao_);
      // load data into vertex buffers
      glBindBuffer(GL_ARRAY_BUFFER, vbo_);
      glBufferData(GL_ARRAY_BUFFER, vertices.size() * 3 * sizeof(float), &vertices[0], GL_STATIC_DRAW);

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

      // set the vertex attribute pointers
      // vertex Positions
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    }
  };

} // namespace owds

#endif // OWDS_GRAPHICS_OPENGL_LINESHANDLE_H
