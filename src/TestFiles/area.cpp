#include <iostream>
#include <vector>

#include "overworld/Geometry/Polygon.h"


int main(int argc, char** argv)
{
  owds::Polygon square({{1,1}, {1,3}, {3,3}, {3,1}});
  owds::Polygon u_shape({{2,2}, {-1,2}, {-1,6}, {2,6}, {2,5}, {1,5}, {1,4}, {2,4}});

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

  return 0;
}