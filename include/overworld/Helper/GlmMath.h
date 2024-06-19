#ifndef OWDS_HELPER_GLMMATH_H
#define OWDS_HELPER_GLMMATH_H

#include <glm/gtc/type_ptr.hpp>
#include <cstring>

namespace owds {
  inline glm::vec3 ToV3(const std::array<float, 3>& in)
  {
    glm::vec3 out;
    std::memcpy(glm::value_ptr(out), std::addressof(in), sizeof(float) * 3);
    return out;
  }

  inline glm::mat4 ToM4(const std::array<float, 16>& in)
  {
    glm::mat4 out;
    std::memcpy(glm::value_ptr(out), std::addressof(in), sizeof(float) * 16);
    return out;
  }

  inline std::array<float, 3> FromV3(const glm::vec3& in)
  {
    std::array<float, 3> out;
    std::memcpy(std::addressof(out), glm::value_ptr(in), sizeof(float) * 3);
    return out;
  }

  inline std::array<float, 16> FromM4(const glm::mat4& in)
  {
    std::array<float, 16> out;
    std::memcpy(std::addressof(out), glm::value_ptr(in), sizeof(float) * 16);
    return out;
  }
}

#endif // OWDS_HELPER_GLMMATH_H
