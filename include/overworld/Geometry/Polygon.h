#ifndef OWDS_POLYGON_H
#define OWDS_POLYGON_H

#include <vector>

namespace owds {

struct point_t
{
  point_t(double point_x, double point_y) : x(point_x), y(point_y) {}

  double x;
  double y;
};

struct segement_t
{
  segement_t(double seg_a, double seg_m, double seg_c) : a(seg_a), m(seg_m), c(seg_c) {}

	double a;
	double m;
	double c;
};

class Polygon
{
public:
  explicit Polygon(const std::vector<point_t>& poly_points) : points(poly_points) {}

  int isInside(const point_t& p, const std::vector<point_t>& vertex);
  std::vector<segement_t> offsetingPolygon(double offset, const std::vector<point_t>& vertices);
  void extractVectrices(const std::vector<segement_t>& segements_out, const std::vector<segement_t>& segements_in,
                        std::vector<point_t>& inner, std::vector<point_t>& outer,
                        const std::vector<point_t>& base);

  int isInside(const point_t& p) { return isInside(p, points); }
  std::vector<segement_t> offsetingPolygon(double offset) { return offsetingPolygon(offset, points); }
  void extractVectrices(const std::vector<segement_t>& segements_out, const std::vector<segement_t>& segements_in,
                        std::vector<point_t>& inner, std::vector<point_t>& outer)
  {
    return extractVectrices(segements_out, segements_in, inner, outer, points);
  }

  std::vector<point_t> points;
  
private:
  point_t getIntersect(const segement_t& seg1, const segement_t& seg2);
  void getInOutPoints(const std::vector<point_t>& scare, const std::vector<point_t>& base, point_t& in, point_t& out);
};

} // namespace owds

#endif // OWDS_POLYGON_H