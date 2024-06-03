#ifndef OWDS_POLYGON_H
#define OWDS_POLYGON_H

#include <vector>

#include "overworld/Geometry/Pose.h"

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
    explicit Polygon(const std::vector<point_t>& poly_points, double hysteresis = 0.0001);

    void setHysteresis(double hysteresis);

    void transformIn(const Pose& pose);

    int isInside(const point_t& p) const { return isInside(p, points_); }
    int isInsideInner(const point_t& p) const { return isInside(p, inner_points_); }
    int isInsideOuter(const point_t& p) const { return isInside(p, outer_points_); }

    const std::vector<point_t>& getBasePoints() const { return base_points_; }
    const std::vector<point_t>& getPoints() const { return points_; }
    const std::vector<point_t>& getInnerPoints() const { return inner_points_; }
    const std::vector<point_t>& getOuterPoints() const { return outer_points_; }

  private:
    std::vector<point_t> base_points_;
    Pose last_transform_pose_;
    double hysteresis_;

    std::vector<point_t> inner_points_;
    std::vector<point_t> points_;
    std::vector<point_t> outer_points_;

    point_t getIntersect(const segement_t& seg1, const segement_t& seg2);
    void getInOutPoints(const std::vector<point_t>& scare, const std::vector<point_t>& base, point_t& in, point_t& out);

    void computeInOutPolygons();

    int isInside(const point_t& p, const std::vector<point_t>& vertex) const;
    std::vector<segement_t> offsetingPolygon(double offset, const std::vector<point_t>& vertices);
    void extractVectrices(const std::vector<segement_t>& segments_out, const std::vector<segement_t>& segments_in,
                          std::vector<point_t>& inner, std::vector<point_t>& outer,
                          const std::vector<point_t>& base);

    std::vector<segement_t> offsetingPolygon(double offset) { return offsetingPolygon(offset, points_); }
    void extractVectrices(const std::vector<segement_t>& segments_out, const std::vector<segement_t>& segments_in,
                          std::vector<point_t>& inner, std::vector<point_t>& outer)
    {
      extractVectrices(segments_out, segments_in, inner, outer, points_);
    }
  };

} // namespace owds

#endif // OWDS_POLYGON_H