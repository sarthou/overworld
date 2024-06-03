#include <iostream>
#include <vector>

#include "overworld/Geometry/Polygon.h"

int main(int argc, char** argv)
{
  owds::Polygon square({
    {1, 1},
    {1, 3},
    {3, 3},
    {3, 1}
  });
  owds::Polygon u_shape({
    {2,  2},
    {-1, 2},
    {-1, 6},
    {2,  6},
    {2,  5},
    {1,  5},
    {1,  4},
    {2,  4}
  });

  owds::point_t M(1.5, 0.5);
  owds::point_t N(1.5, 2.5);
  owds::point_t O(1.5, 4.5);
  owds::point_t P(1.5, 5.5);

  std::cout << "square and M = " << square.isInside(M) << std::endl;
  std::cout << "square and N = " << square.isInside(N) << std::endl;
  std::cout << "square and O = " << square.isInside(O) << std::endl;
  std::cout << "square and P = " << square.isInside(P) << std::endl;

  std::cout << "u_shape and M = " << u_shape.isInside(M) << std::endl;
  std::cout << "u_shape and N = " << u_shape.isInside(N) << std::endl;
  std::cout << "u_shape and O = " << u_shape.isInside(O) << std::endl;
  std::cout << "u_shape and P = " << u_shape.isInside(P) << std::endl;

  owds::Polygon simple({
    {-1, -1},
    {-1, 1 },
    {1,  1 },
    {1,  -1}
  });

  owds::Pose p1(std::array<double, 3>{2., 2., 2.}, std::array<double, 3>{45. * M_PI / 180., 30. * M_PI / 180., 10. * M_PI / 180.});
  std::cout << "yaw = " << p1.getYaw() * 180. / M_PI << std::endl;
  std::cout << "roll = " << p1.getRoll() * 180. / M_PI << std::endl;
  std::cout << "pitch = " << p1.getPitch() * 180. / M_PI << std::endl;
  std::cout << "pan = " << p1.getOriginPan() * 180. / M_PI << std::endl;
  std::cout << "tilt = " << p1.getOriginTilt() * 180. / M_PI << std::endl;
  square.transformIn(p1);

  for(auto& p : square.getPoints())
    std::cout << p.x << " -- " << p.y << std::endl;

  return 0;
}