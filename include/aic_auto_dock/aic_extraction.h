#ifndef AIC_EXTRACTION_H
#define AIC_EXTRACTION_H

#include "aic_auto_dock/math/status.h"
#include "aic_laser_feature_line_extraction/LineSegmentList.h"
#include "aic_laser_feature_line_extraction/line_extraction_ros.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class AicExtractionInterface
{
public:
  AicExtractionInterface() {}
  std::string tag;
  std::string name;
};

class AicExtractionPolygon
{
public:
  AicExtractionPolygon() {}
  std::string frame_id;
  geometry_msgs::Polygon extra_polygon;
};

class AicExtraction
{
public:
  AicExtraction(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~AicExtraction();

  line_extraction::LineExtractionROS* line_extractor_;

  virtual bool run(const geometry_msgs::Pose& old_middle_point, geometry_msgs::Pose& new_middle_point) = 0;

  void setLineExtractor(line_extraction::LineExtractionROS* line_extractor);
  virtual bool setInterface(const AicExtractionInterface& interface) = 0;
  bool setExtraPolygon(const AicExtractionPolygon& polygon);
  void setRoughRecgonizeMark(bool roughRecgonize_mark);
  void setFarDetection(bool);
  void setDirection(const docking_direction&);
  void setUnSubscribe();

  void timeSynchronizerA1Callback(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                  const nav_msgs::OdometryConstPtr& odom_msg);
  void timeSynchronizerA2Callback(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                  const nav_msgs::OdometryConstPtr& odom_msg);
  void timeSynchronizerACallback(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                 const nav_msgs::OdometryConstPtr& odom_msg);

protected:
  ros::NodeHandle nh_, nh_local_;

  typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::LaserScan, nav_msgs::Odometry > MySyncPolicyA;
  message_filters::Subscriber< sensor_msgs::LaserScan >*scan_sub_, *scan_sub1_, *scan_sub2_;
  message_filters::Subscriber< nav_msgs::Odometry >* odom_sub_;
  message_filters::Synchronizer< MySyncPolicyA >*syncA_, *syncA1_, *syncA2_;

  boost::recursive_mutex mutex_;

  bool receive_synchron_msg_ = false;
  sensor_msgs::LaserScan::ConstPtr scan_msg_;
  tf::StampedTransform odom_foot_frame_;

  AicExtractionInterface interface_;
  AicExtractionPolygon polygon_;
  bool roughRecgonize_mark_;

  docking_direction dock_direction_;
  std::string scan_frame_, scan_topic_, scan_frame1_, scan_topic1_, scan_frame2_, scan_topic2_, scan_frame_single_,
      scan_topic_single_;
  bool multi_scan_;
  bool front_scan_mark_, back_scan_mark_, scan_mark_;
  bool far_detection_ = false;
};

#endif // AIC_EXTRACTION_H
