#ifndef OWDS_DEBUGLINE_H
#define OWDS_DEBUGLINE_H

#include <glm/vec3.hpp>
#include <vector>

namespace owds {

  class DebugLine
  {
  public:
    DebugLine(const std::vector<glm::vec3>& vertices,
              const std::vector<unsigned int>& indices,
              const glm::vec3& color,
              double remaining_time = 0) : id_(id_counter++),
                                           vertices_(vertices),
                                           indices_(indices),
                                           color_(color),
                                           remaining_time_(remaining_time)
    {
    }

    unsigned int id_;
    std::vector<glm::vec3> vertices_;
    std::vector<unsigned int> indices_;
    glm::vec3 color_;
    double remaining_time_ = 0;

  private:
    static unsigned int id_counter;
  };

} // namespace owds

#endif // OWDS_DEBUGLINE_H
