#ifndef AICPOLY_H
#define AICPOLY_H

#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

using namespace std;

namespace aicpoly
{
/**********/
//排斥实验
inline bool IsRectCross(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2,
                        const geometry_msgs::Point32& q1, const geometry_msgs::Point32& q2)
{
  bool ret = min(p1.x, p2.x) <= max(q1.x, q2.x) && min(q1.x, q2.x) <= max(p1.x, p2.x) &&
             min(p1.y, p2.y) <= max(q1.y, q2.y) && min(q1.y, q2.y) <= max(p1.y, p2.y);
  return ret;
}

inline bool IsLineSegmentCross(geometry_msgs::Point32 a, geometry_msgs::Point32 b, geometry_msgs::Point32 c,
                               geometry_msgs::Point32 d)
{
  const double eps = 1e-6;
  double h, i, j, k;
  h = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  i = (b.x - a.x) * (d.y - a.y) - (b.y - a.y) * (d.x - a.x);
  j = (d.x - c.x) * (a.y - c.y) - (d.y - c.y) * (a.x - c.x);
  k = (d.x - c.x) * (b.y - c.y) - (d.y - c.y) * (b.x - c.x);
  return h * i <= eps && j * k <= eps;
}

inline bool GetCrossPoint(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2,
                          const geometry_msgs::Point32& q1, const geometry_msgs::Point32& q2) //,long &x,long &y
{
  if (IsRectCross(p1, p2, q1, q2))
    if (IsLineSegmentCross(p1, p2, q1, q2))
      return true;
    else
      return false;
  return false;
}

inline bool GetCrossPolygon(const geometry_msgs::Polygon& test_lines, const geometry_msgs::Polygon& polygon)
{
  // core algorithm
  size_t i_polygon, j_polygon;
  bool c = false;
  for (vector< geometry_msgs::Point32 >::const_iterator cit = test_lines.points.begin();
       cit < test_lines.points.end() - 1; cit++)
  {
    for (i_polygon = 0, j_polygon = polygon.points.size() - 1; i_polygon < polygon.points.size();
         j_polygon = i_polygon++)
    {
      if (GetCrossPoint(polygon.points[i_polygon], polygon.points[j_polygon], *cit, *(cit + 1)))
      {
        c = true;
        break;
      }
    }

    if (c)
      break;
  }
}

/**********/
inline float GetCross(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2,
                      const geometry_msgs::Point32& p)
{
  return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

inline bool point_in_polygon(const geometry_msgs::Point32& p, const geometry_msgs::Polygon& polygon)
{
  size_t i, j, c;
  for (i = 0, j = polygon.points.size() - 1; i < polygon.points.size() - 1; j = i++)
  {
    if (GetCross(polygon.points[j], polygon.points[i], p) * GetCross(polygon.points[i], polygon.points[i + 1], p) < 0)
      return false;
  }
  return true;
}

inline bool lines_in_polygon(const geometry_msgs::Polygon& test_lines, const geometry_msgs::Polygon& polygon)
{
  bool c = false;
  for_each(test_lines.points.begin(), test_lines.points.end(), [&](const geometry_msgs::Point32& pt) {
    if (point_in_polygon(pt, polygon))
      c = true;
  });
  return c;
}

/***********/
inline double distance(const geometry_msgs::Point32 p, const geometry_msgs::Point32 p1)
{
  return sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2));
}
}

#endif // AICPOLY_H
