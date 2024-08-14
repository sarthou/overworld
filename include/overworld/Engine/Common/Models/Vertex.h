#ifndef OWDS_COMMON_VERTEX_H
#define OWDS_COMMON_VERTEX_H

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace owds {

  class Vertex
  {
  public:
    // Never change the attributes order
    glm::vec3 position_;
    glm::vec3 normal_;
    glm::vec2 uv_;
    glm::vec3 tangent_;
    glm::vec3 bitangent_;
  };

} // namespace owds

#endif // OWDS_COMMON_VERTEX_H
