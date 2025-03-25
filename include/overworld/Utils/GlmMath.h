#ifndef OWDS_HELPER_GLMMATH_H
#define OWDS_HELPER_GLMMATH_H

#include <cstring>
#include <glm/gtc/type_ptr.hpp>

namespace owds {
  inline glm::vec3 toV3(const std::array<float, 3>& in)
  {
    glm::vec3 out;
    std::memcpy(glm::value_ptr(out), std::addressof(in), sizeof(float) * 3);
    return out;
  }

  inline glm::quat toQt(const std::array<float, 4>& in)
  {
    glm::quat out;
    std::memcpy(glm::value_ptr(out), std::addressof(in), sizeof(float) * 4);
    return out;
  }

  inline glm::vec3 toV3(const std::array<double, 3>& in)
  {
    std::array<float, 3> array{(float)in.at(0), (float)in.at(1), (float)in.at(2)};
    return toV3(array);
  }

  inline glm::quat toQt(const std::array<double, 4>& in)
  {
    std::array<float, 4> array{(float)in.at(0), (float)in.at(1), (float)in.at(2), (float)in.at(3)};
    return toQt(array);
  }

  inline glm::mat4 toM4(const std::array<float, 16>& in)
  {
    glm::mat4 out;
    std::memcpy(glm::value_ptr(out), std::addressof(in), sizeof(float) * 16);
    return out;
  }

  inline std::array<float, 3> fromV3(const glm::vec3& in)
  {
    std::array<float, 3> out;
    std::memcpy(std::addressof(out), glm::value_ptr(in), sizeof(float) * 3);
    return out;
  }

  inline std::array<float, 16> fromM4(const glm::mat4& in)
  {
    std::array<float, 16> out;
    std::memcpy(std::addressof(out), glm::value_ptr(in), sizeof(float) * 16);
    return out;
  }
} // namespace owds

#endif // OWDS_HELPER_GLMMATH_H
