#include "aic_auto_dock/math/recognizeUtility.h"

namespace recognize_extraction
{

double gaussian_kernel(double distance, double bandwidth)
{
  return ((1 / (bandwidth * sqrt(2 * M_PI))) * exp(-0.5 * pow(distance / bandwidth, 2)));
}

Point mean_shift(vector< Point > data)
{
  Point iter_position;
  double kernel_bandwidth = 25;

  Point mean_point = {0, 0};
  for (vector< Point >::iterator cit = data.begin(); cit < data.end(); cit++)
  {
    mean_point.x += (*cit).x;
    mean_point.y += (*cit).y;
  }
  mean_point.x = mean_point.x / data.size();
  mean_point.y = mean_point.y / data.size();

  int n_iterations = 5;
  for (int i = 0; i < n_iterations; i++)
  {
    Point numerator = {0, 0};
    double denominator = 0;
    for (vector< Point >::iterator cit = data.begin(); cit < data.end(); cit++)
    {
      double dist = distance(*cit, mean_point);
      double weight = gaussian_kernel(dist, kernel_bandwidth);
      numerator.x += weight * (*cit).x;
      numerator.y += weight * (*cit).y;
      denominator += weight;
    }
    numerator.x = numerator.x / denominator;
    numerator.y = numerator.y / denominator;
    iter_position = numerator;
  }
  return iter_position;
}

tf::Transform extra_filter(vector< tf::Transform > vec_transform)
{
  ROS_WARN("vec_transform size:%d", vec_transform.size());
  if (vec_transform.size() == 1)
    return vec_transform.front();

  /*** 计算均值 ***/
  geometry_msgs::Pose2D mean_pose2d;
  vector< double > vec_angels;
  for (vector< tf::Transform >::iterator cit = vec_transform.begin(); cit < vec_transform.end(); cit++)
  {
    double roll, pitch, yaw;
    tf::Matrix3x3((*cit).getRotation()).getRPY(roll, pitch, yaw);
    mean_pose2d.x += (*cit).getOrigin().getX();
    mean_pose2d.y += (*cit).getOrigin().getY();
    vec_angels.push_back(yaw);
    ROS_DEBUG_NAMED("docking", "x:%f, y:%f, yaw:%f", (*cit).getOrigin().getX(), (*cit).getOrigin().getY(), yaw);
  }

  mean_pose2d.x = mean_pose2d.x / vec_transform.size();
  mean_pose2d.y = mean_pose2d.y / vec_transform.size();
  mean_pose2d.theta = angles::normalize_angle(getAverageAngle(vec_angels));
  ROS_DEBUG_NAMED("docking", "average:: size:%d, x:%f, y:%f, yaw:%f", vec_transform.size(), mean_pose2d.x,
                  mean_pose2d.y, mean_pose2d.theta);

  /*** 除去非法的角度 ***/
  for (int i = 0; i < vec_transform.size(); i++)
  {
    double roll, pitch, yaw;
    tf::Matrix3x3(vec_transform.at(i).getRotation()).getRPY(roll, pitch, yaw);
    double delta_angle = fabs(getDiffAngle(mean_pose2d.theta, yaw));
    if (delta_angle > 0.05)
    {
      vec_transform.erase(vec_transform.begin() + i);
      ROS_DEBUG_NAMED("docking", "illegal theta erase:%d", i);
    }
  }

  /*** return ***/
  if (vec_transform.size() == 0)
  {
    ROS_DEBUG_NAMED("docking", "illegal theta Used full vector to calculate average pt");
    tf::Quaternion q(0, 0, mean_pose2d.theta);
    tf::Transform mean_frame(q, tf::Vector3(mean_pose2d.x, mean_pose2d.y, 0));
    return mean_frame;
  }

  /*** 计算平均欧式距离 ***/
  double ave_dist = 0.0;
  for (int i = 0; i < vec_transform.size(); i++)
    ave_dist += sqrt(pow(vec_transform.at(i).getOrigin().getX() - mean_pose2d.x, 2) +
                     pow(vec_transform.at(i).getOrigin().getY() - mean_pose2d.y, 2));
  ave_dist = ave_dist / vec_transform.size();

  /*** 出去非法的距离 ***/
  for (int i = 0; i < vec_transform.size(); i++)
  {
    double dist = sqrt(pow(vec_transform.at(i).getOrigin().getX() - mean_pose2d.x, 2) +
                       pow(vec_transform.at(i).getOrigin().getY() - mean_pose2d.y, 2));

    if (fabs(ave_dist - dist) > 0.01)
    {
      vec_transform.erase(vec_transform.begin() + i);
      ROS_DEBUG_NAMED("docking", "illegal distance erase:%d", i);
    }
  }

  ROS_WARN("vec_transform size:%d", vec_transform.size());

  /*** return ***/
  if (vec_transform.size() == 0)
  {
    ROS_DEBUG_NAMED("docking", "illegal distance Used full vector to calculate average pt");
    tf::Quaternion q;
    q.setRPY(0, 0, mean_pose2d.theta);
    tf::Transform mean_frame(q, tf::Vector3(mean_pose2d.x, mean_pose2d.y, 0));
    return mean_frame;
  }

  /*** 计算均值 ***/
  mean_pose2d.x = 0;
  mean_pose2d.y = 0;
  mean_pose2d.theta = 0;
  vec_angels.clear();
  for (vector< tf::Transform >::iterator cit = vec_transform.begin(); cit < vec_transform.end(); cit++)
  {
    double roll, pitch, yaw;
    tf::Matrix3x3((*cit).getRotation()).getRPY(roll, pitch, yaw);
    mean_pose2d.x += (*cit).getOrigin().getX();
    mean_pose2d.y += (*cit).getOrigin().getY();
    vec_angels.push_back(yaw);
  }

  mean_pose2d.x = mean_pose2d.x / vec_transform.size();
  mean_pose2d.y = mean_pose2d.y / vec_transform.size();
  mean_pose2d.theta = angles::normalize_angle(getAverageAngle(vec_angels));
  ROS_DEBUG_NAMED("docking", "average:: size:%d, x:%f, y:%f, yaw:%f", vec_transform.size(), mean_pose2d.x,
                  mean_pose2d.y, mean_pose2d.theta);

  /*** output ***/
  tf::Quaternion q;
  q.setRPY(0, 0, mean_pose2d.theta);
  tf::Transform mean_frame(q, tf::Vector3(mean_pose2d.x, mean_pose2d.y, vec_transform.front().getOrigin().z()));
  return mean_frame;
}

double getAverageAngle(const vector< double >& vec_angles)
{
  double y = 0.0, x = 0.0, angle = 0.0;
  for_each(vec_angles.begin(), vec_angles.end(), [&](double a) {
    y += sin(angles::normalize_angle_positive(a));
    x += cos(angles::normalize_angle_positive(a));
  });
  y /= vec_angles.size();
  x /= vec_angles.size();
  angle = atan2(y, x);

  //  if (x != 0)
  //  {
  //    angle = atan(y / x);
  //    if (y > 0 && x > 0)
  //      angle = angle;
  //    else if (x < 0)
  //      angle += M_PI;
  //    else if (y < 0 && x > 0)
  //      angle += 2 * M_PI;
  //  }
  //  else
  //    angle = M_PI_2;

  return (angles::normalize_angle(angle));
}

double getDiffAngle(const double& angle1, const double& angle2)
{
  double diff_angle = angles::normalize_angle_positive(angle1) - angles::normalize_angle_positive(angle2);
  if (diff_angle > M_PI)
    diff_angle -= 2 * M_PI;
  else if (diff_angle < -M_PI)
    diff_angle += 2 * M_PI;
  return diff_angle;
}
}
