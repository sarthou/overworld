#ifndef OWDS_DEBUGTEXT_H
#define OWDS_DEBUGTEXT_H

#include <glm/vec3.hpp>
#include <string>

namespace owds {

  class Actor;

  struct DebugText_t
  {
    std::string text;
    bool centered = false;
    glm::vec3 position;
    glm::vec3 color;
    float height = 0.2;
    double remaining_time = 0;
    Actor* linked_actor = nullptr;
  };

} // namespace owds

#endif // OWDS_DEBUGTEXT_H