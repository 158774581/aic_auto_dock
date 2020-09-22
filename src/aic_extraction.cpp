#include "aic_auto_dock/aic_extraction.h"

AicExtraction::AicExtraction(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
{
  nh_local_.param< bool >("multi_scan", multi_scan_, true);
}

AicExtraction::~AicExtraction() {}

void AicExtraction::setLineExtractor(line_extraction::LineExtractionROS* line_extractor)
{
  line_extractor_ = line_extractor;
}

bool AicExtraction::setExtraPolygon(const AicExtractionPolygon& polygon)
{
  polygon_ = polygon;
  if (polygon_.frame_id != "odom" && polygon_.frame_id != "base_footprint")
  {
    ROS_ERROR("error input polygon");
    return false;
  }
  return true;
}

void AicExtraction::setRoughRecgonizeMark(bool roughRecgonize_mark) { roughRecgonize_mark_ = roughRecgonize_mark; }

void AicExtraction::setFarDetection(bool far_detection) { far_detection_ = far_detection; }

void AicExtraction::setDirection(const docking_direction& dock_direction)
{
  dock_direction_ = dock_direction;
  if (multi_scan_)
  {
    if (dock_direction_ == docking_direction::forward)
    {
      scan_topic_ = scan_topic1_;
      scan_frame_ = scan_frame1_;
      front_scan_mark_ = true;
    }
    else if (dock_direction_ == docking_direction::backward)
    {
      scan_topic_ = scan_topic2_;
      scan_frame_ = scan_frame2_;
      back_scan_mark_ = true;
    }
    else
    {
      scan_topic_ = scan_topic1_;
      scan_frame_ = scan_frame1_;
      front_scan_mark_ = true;
    }
  }
  else
  {
    scan_topic_ = scan_topic_single_;
    scan_frame_ = scan_frame_single_;
    scan_mark_ = true;
  }
  line_extractor_->setScanFrame(scan_frame_);
}

void AicExtraction::setUnSubscribe()
{
  front_scan_mark_ = false;
  back_scan_mark_ = false;
  scan_mark_ = false;
  scan_msg_ = nullptr;
}

void AicExtraction::timeSynchronizerA1Callback(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                              const nav_msgs::OdometryConstPtr& odom_msg)
{
  boost::unique_lock< boost::recursive_mutex > lock(mutex_);

  if(front_scan_mark_)
  {
    scan_msg_ = laser_msg;
    tf::poseMsgToTF(odom_msg->pose.pose, odom_foot_frame_);
    receive_synchron_msg_ = true;
    //  ROS_INFO("sychronize:%d", receive_synchron_msg_);
  }

  lock.unlock();
}

void AicExtraction::timeSynchronizerA2Callback(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                              const nav_msgs::OdometryConstPtr& odom_msg)
{
  boost::unique_lock< boost::recursive_mutex > lock(mutex_);

  if(back_scan_mark_)
  {
    scan_msg_ = laser_msg;
    tf::poseMsgToTF(odom_msg->pose.pose, odom_foot_frame_);
    receive_synchron_msg_ = true;
    //  ROS_INFO("sychronize:%d", receive_synchron_msg_);
  }

  lock.unlock();
}

void AicExtraction::timeSynchronizerACallback(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                              const nav_msgs::OdometryConstPtr& odom_msg)
{
  boost::unique_lock< boost::recursive_mutex > lock(mutex_);

  if(scan_mark_)
  {
    scan_msg_ = laser_msg;
    tf::poseMsgToTF(odom_msg->pose.pose, odom_foot_frame_);
    receive_synchron_msg_ = true;
    //  ROS_INFO("sychronize:%d", receive_synchron_msg_);
  }

  lock.unlock();
}
