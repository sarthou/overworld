#ifndef OWDS_WORLDTYPES_H
#define OWDS_WORLDTYPES_H

#include <cstddef>
#include <array>

namespace owds {

struct RaycastHitResult_t
  {
    bool hit;                      // Whether the ray hit something
    std::array<double, 3> position; // Hit position (x, y, z)
    std::array<double, 3> normal;   // Hit normal (x, y, z)
    double distance;                // Distance to the hit
    // std::string actor_name;
    size_t actor_id;
    size_t body_id;

    RaycastHitResult_t() : hit(false), position{0, 0, 0}, normal{0, 0, 0}, distance(0.0f), actor_id(-1) {}
  };

  struct AABB_t
  {
  public:
    std::array<float, 3> min = {0., 0., 0.};
    std::array<float, 3> max = {0., 0., 0.};

    AABB_t() : valid_(false) {}
    AABB_t(const std::array<float, 3>& aabb_min,
           const std::array<float, 3>& aabb_max) : min(aabb_min),
                                                   max(aabb_max), valid_(true) {}

    bool isValid() const { return valid_; }

  private:
    bool valid_;
  };

  struct ContactPoint_t
  {
    std::array<float, 3> position; // World-space position of the contact
    std::array<float, 3> normal;   // Normal at the contact point
    float separation;              // Separation distance (negative for penetration)
    float impulse;                 // Normal impulse applied at the contact point
  };

} // namespace owds

#endif // OWDS_WORLDTYPES_H