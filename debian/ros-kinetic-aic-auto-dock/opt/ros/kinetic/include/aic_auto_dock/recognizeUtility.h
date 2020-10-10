#ifndef RECOGNIZEUTILITY_H
#define RECOGNIZEUTILITY_H

#include <angles/angles.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <string>
#include <tf/tf.h>
#include <vector>

namespace recognize_extraction
{
using namespace std;

typedef struct Point
{
  Point() {}
  Point(double xm, double ym)
  {
    x = xm;
    y = ym;
  }
  double x;
  double y;
} Point;

struct actionlibParam
{
  float x_max = 0.0;
  float x_min = 0.0;
  float y_max = 0.0;
  float y_min = 0.0;
  int average_loop = 0;
  float calibrationAngleDiff = 0.0;
  float calibrationDistDiff = 0.0;
  bool useCombination = false;
};

struct extra_line
{
  float start_x;
  float start_y;
  float end_x;
  float end_y;
  double angle;
  int num;
};

struct LineParam
{
  LineParam() {}
  float k;
  float b;
};

enum RecognizeMode
{
  precise_recognize,
  rough_recognize
};

/**
 * @brief getLineParam 计算直线斜率
 */
inline void getLineParam(float& x1, float& y1, float& x2, float& y2, LineParam& LP)
{
  double mm = 0;

  mm = x2 - x1;

  if (0 == mm)
  {
    LP.k = 10000.0;
    LP.b = y1 - LP.k * x1;
  }
  else
  {
    LP.k = (y2 - y1) / (x2 - x1);
    LP.b = y1 - LP.k * x1;
  }
}

/**
 * @brief getCross 计算两条直线的交点
 * @return
 */
inline bool getCross(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4,
                     vector< geometry_msgs::Pose2D >& pts)
{
  geometry_msgs::Pose2D pt;
  LineParam para1, para2;
  getLineParam(x1, y1, x2, y2, para1);
  getLineParam(x3, y3, x4, y4, para2);

  //  std::cout << "para1.k: "<< para1.k << "\tpara1.b: "<< para1.b << "\tpara2.k: " << para2.k << "\tpara2.b: " <<
  //  para2.b << std::endl;

  // calculate whether it is parallel
  if (fabs(para1.k - para2.k) > 0.05)
  {
    pt.x = (para2.b - para1.b) / (para1.k - para2.k);
    pt.y = para1.k * pt.x + para1.b;

    pts.push_back(pt);
    return true;
  }
  else
  {
    return false;
  }
}

inline void middle_pt(const geometry_msgs::Pose2D& pt1, const geometry_msgs::Pose2D& pt2, geometry_msgs::Pose2D& pt)
{
  pt.x = (pt1.x + pt2.x) / 2.0;
  pt.y = (pt1.y + pt2.y) / 2.0;
}

inline double DegToRad(double deg) { return deg * M_PI / 180; }
inline double RadToDeg(double rad) { return rad * 180 / M_PI; }

inline double distPtLine(const Point& pt1, const Point& pt2, const Point& pt3)
{

  float dis = 0.f;

  float a_x = pt3.x - pt1.x;
  float a_y = pt3.y - pt1.y;

  float b_x = pt2.x - pt1.x;
  float b_y = pt2.y - pt1.y;

  float k = (a_x * b_x + a_y * b_y) / (b_x * b_x + b_y * b_y);

  float c_x = k * b_x;
  float c_y = k * b_y;

  float e_x = a_x - c_x;
  float e_y = a_y - c_y;

  return sqrt(pow(e_x, 2) + pow(e_y, 2));
}

/***
 * 计算直线与X轴的夹角
 ***/
inline float getAngelOfXaxis(Point& a,
                             Point& b) //直线与X轴的夹角。现在使用的是直线与X轴正半轴的夹角范围[-90,90]，线段没有方向。
{
  float rota = std::atan2((b.y - a.y), (b.x - a.x));
  if (rota > M_PI / 2)
    rota = -M_PI + rota;
  else if (rota < -M_PI / 2)
    rota = M_PI + rota;
  return rota;
}

inline double distance(Point p, Point p1) { return sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2)); }

double gaussian_kernel(double, double);
Point mean_shift(vector< Point >);
tf::Transform extra_filter(vector< tf::Transform >);

/***
 * 计算直线与X轴的夹角
 ***/
inline float
getAngelOfXaxisM_PI(Point& a, Point& b, Point& c,
                    Point& d) //直线与X轴的夹角。现在使用的是直线与X轴正半轴的夹角范围[-180,180]，线段没有方向。
{
  Point vecA = {b.x - a.x, b.y - a.y};
  Point vecB = {d.x - c.x, d.y - c.y};
  double rota = acos((vecA.x * vecB.x + vecA.y * vecB.y) / (distance(a, b) * distance(c, d)));
  if (vecA.y < 0)
    rota = -rota;
  return rota;
}

/***
 * 计算两个向量的夹角
 ***/
inline float
getAngelOfVectorM_PI(Point& a, Point& b, Point& c,
                    Point& d) //两个向量的夹角，夹角范围[-180,180]，向量有方向。
{
  Point vecA = {b.x - a.x, b.y - a.y};
  Point vecB = {d.x - c.x, d.y - c.y};
  double rota = acos((vecA.x * vecB.x + vecA.y * vecB.y) / (distance(a, b) * distance(c, d)));
  if (vecA.y < 0)
    rota = -rota;
  return rota;
}


double getAverageAngle(const vector< double >& vec_angles);
double getDiffAngle(const double& angle1, const double& angle2);
}
#endif // UTILITY_H
