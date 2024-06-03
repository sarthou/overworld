#include "overworld/Geometry/Polygon.h"

#include <cmath>

namespace owds {

  Polygon::Polygon(const std::vector<point_t>& poly_points, double hysteresis) : points_(poly_points),
                                                                                 base_points_(poly_points),
                                                                                 hysteresis_(hysteresis)
  {
    computeInOutPolygons();
  }

  void Polygon::setHysteresis(double hysteresis)
  {
    hysteresis_ = hysteresis;
    computeInOutPolygons();
  }

  void Polygon::transformIn(const Pose& pose)
  {
    if(last_transform_pose_ == pose)
      return;

    last_transform_pose_ = pose;
    points_.clear();
    points_.reserve(base_points_.size());

    double sin_angle = std::sin(pose.getYaw());
    double cos_angle = std::cos(pose.getYaw());

    for(auto& point : base_points_)
    {
      double x_new = point.x * cos_angle - point.y * sin_angle;
      double y_new = point.x * sin_angle + point.y * cos_angle;
      points_.emplace_back(x_new + pose.getX(), y_new + pose.getY());
    }

    computeInOutPolygons();
  }

  int Polygon::isInside(const point_t& p, const std::vector<point_t>& vertex) const
  {
    int cross = 0;
    for(int i = 0, j = vertex.size() - 1; i < vertex.size(); j = i++)
    {
      if(((vertex[i].y > p.y) != (vertex[j].y > p.y)) &&
         (p.x < (vertex[j].x - vertex[i].x) * (p.y - vertex[i].y) / (vertex[j].y - vertex[i].y) + vertex[i].x))
        cross = !cross;
    }
    return cross;
  }

  std::vector<segement_t> Polygon::offsetingPolygon(double offset, const std::vector<point_t>& vertices)
  {
    std::vector<segement_t> segments;
    segments.reserve(vertices.size());
    std::vector<point_t>::const_iterator it_next;

    for(auto it_vect = vertices.begin(); it_vect < vertices.end(); ++it_vect)
    {
      double x1 = it_vect->x;
      double y1 = it_vect->y;

      // find the next point
      if(it_vect + 1 == vertices.end())
        it_next = vertices.begin();
      else
        it_next = it_vect + 1;

      double a, m, c;
      double x2 = it_next->x;
      double y2 = it_next->y;
      // finds the parameters of line's equations. The equation line has the form a*y = m*x + c. where slope is m/a and intercept is c/a .
      if(x2 != x1)
      {
        a = 1;
        m = (y2 - y1) / (x2 - x1);
        c = y1 - m * x1;
      }
      else
      {
        a = 0;
        m = 1;
        c = -x1;
      }
      if(it_vect + 2 == vertices.end())
        it_next = vertices.begin();
      else if(it_vect + 1 == vertices.end())
        it_next = vertices.begin() + 1;
      else
        it_next = it_vect + 2;

      double x3 = it_next->x;
      double y3 = it_next->y;

      if(m == 0)
      {
        if(y3 > c) // shifts the line by changing the intercept to get edge of inner polygon
          c = c + offset;
        else // shifts the line by changing the intercept to get edge of inner polygon
          c = c - offset;
      }
      else // if(m != 0)
      {
        double x;
        double c_tmp;
        x = (a * y3 - c) / m; // x is calculated to find the direction in which line is to be shifted
                              // shifts the line by changing the intercept to get edge of inner polygon

        double dist = sqrt(m * m + a * a);
        if(x3 > x)
        {
          if(-c / m > 0)
            c_tmp = (abs(c / dist) + offset) * dist;
          else
            c_tmp = (abs(c / dist) - offset) * dist;
        }
        else
        {
          if(-c / m > 0)
            c_tmp = (abs(c / dist) - offset) * dist;
          else
            c_tmp = (abs(c / dist) + offset) * dist;
        }

        if(c < 0)
          c_tmp = -c_tmp;
        c = c_tmp;
      } // end m != 0

      segments.emplace_back(a, m, c);
    } // end for it_vect
    return segments;
  }

  void Polygon::extractVectrices(const std::vector<segement_t>& segments_out, const std::vector<segement_t>& segments_in,
                                 std::vector<point_t>& inner, std::vector<point_t>& outer,
                                 const std::vector<point_t>& base)
  {
    inner.clear();
    inner.reserve(base.size());
    outer.clear();
    outer.reserve(base.size());
    for(unsigned int i = 0, j = segments_out.size() - 1; i < segments_out.size(); j = i++)
    {
      // the order of the points is really important
      //  do not change it !!
      std::vector<point_t> scare = {getIntersect(segments_out[i], segments_out[j]),
                                    getIntersect(segments_out[i], segments_in[j]),
                                    getIntersect(segments_in[i], segments_in[j]),
                                    getIntersect(segments_in[i], segments_out[j])};

      point_t in(0, 0);
      point_t out(0, 0);
      // It will detect witch point is inside or outside of the base polygon
      //  and take the oposite one
      getInOutPoints(scare, base, in, out);
      inner.push_back(in);
      outer.push_back(out);
    }
  }

  point_t Polygon::getIntersect(const segement_t& seg1, const segement_t& seg2)
  {
    double x, y;
    if(seg1.a != 0 && seg2.a != 0)
    {
      x = ((seg2.c / seg2.a) - (seg1.c / seg1.a)) / ((seg1.m / seg1.a) - (seg2.m / seg2.a));
      y = (seg1.m * x + seg1.c) / seg1.a;
    }
    else
    {
      if(seg1.a == 0)
      {
        x = -seg1.c / seg1.m;
        y = (seg2.m * x + seg2.c) / seg2.a;
      }
      else
      {
        x = -seg2.c / seg2.m;
        y = (seg1.m * x + seg1.c) / seg1.a;
      }
    }
    return point_t(x, y);
  }

  void Polygon::getInOutPoints(const std::vector<point_t>& scare, const std::vector<point_t>& base, point_t& in, point_t& out)
  {
    std::vector<int> in_points;
    std::vector<int> out_points;

    for(unsigned int i = 0; i < 4; i++)
    {
      if(isInside(scare[i], base))
        in_points.push_back(i);
      else
        out_points.push_back(i);
    }

    if(in_points.size() == 1)
    {
      in = scare[in_points[0]];
      out = scare[(in_points[0] + 2) % 4];
    }
    else
    {
      out = scare[out_points[0]];
      in = scare[(out_points[0] + 2) % 4];
    }
  }

  void Polygon::computeInOutPolygons()
  {
    // create the inner and outer segments for each segments of the base polygon
    //  /!\ At this point we can say if segments are really inside or ouside the base polygon
    std::vector<segement_t> inner_segments = offsetingPolygon(hysteresis_);
    std::vector<segement_t> outer_segments = offsetingPolygon(-hysteresis_);

    // The vectrice extraction will detect witch is inside and witch is outside the base polygon
    extractVectrices(outer_segments, inner_segments, inner_points_, outer_points_);
  }

} // namespace owds