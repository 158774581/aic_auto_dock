/**************************************************************************
                    _    ___ ____ ____       _
                   / \  |_ _/ ___|  _ \ ___ | |__   ___
                  / _ \  | | |   | |_) / _ \| '_ \ / _ \
                 / ___ \ | | |___|  _ < (_) | |_) | (_) |
                /_/   \_\___\____|_| \_\___/|_.__/ \___/

版权: AICRobo

作者: Liu Yansui

日期: 2018-09-07

描述: 实现了点在多边形内的判断算法

**************************************************************************/

#ifndef PNPOLY_HPP_
#define PNPOLY_HPP_

#include <algorithm>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>
#include <vector>

namespace pnpoly
{
inline bool polygon_contains(const geometry_msgs::Polygon& polygon, double test_x, double test_y)
{
  if (polygon.points.size() < 1)
    return false;
  auto minmax_x = std::minmax_element(
      polygon.points.begin(), polygon.points.end(),
      [&](const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2) { return p1.x < p2.x; });
  auto minmax_y = std::minmax_element(
      polygon.points.begin(), polygon.points.end(),
      [&](const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2) { return p1.y < p2.y; });
  double min_x = minmax_x.first->x;
  double max_x = minmax_x.second->x;
  double min_y = minmax_y.first->y;
  double max_y = minmax_y.second->y;

  // if a point'x is larger than the max x of polygon, it means that this point
  // must be out of this polygon
  if ((test_x > max_x || test_x < min_x) || (test_y > max_y || test_y < min_y))
  {
    return false;
  }

  // core algorithm
  size_t i, j, c = 0;
  for (i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
  {
    if (((polygon.points[i].y > test_y) != (polygon.points[j].y > test_y)) &&
        (test_x < (polygon.points[j].x - polygon.points[i].x) * (test_y - polygon.points[i].y) /
                          (polygon.points[j].y - polygon.points[i].y) +
                      polygon.points[i].x))
    {
      c = !c;
    }
  }
  return c;
}
} // namespace pnpoly
#endif // PNPOLY_HPP_
