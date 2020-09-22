#include "aic_auto_dock/v_shape_extra.h"

VShapeExtra::VShapeExtra(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : AicExtraction(nh, nh_local)
{
  extra_range_marker_ = nh_local_.advertise< visualization_msgs::Marker >("v_shape/extra_range", 10);
  vshape_rectangle_marker_ = nh_local_.advertise< visualization_msgs::Marker >("v_shape/rectangle", 10);
  //  marker_publisher_ = nh_local_.advertise< visualization_msgs::Marker >("v_shape/lines", 10);

  loadParam();

  scan_sub1_ = new message_filters::Subscriber< sensor_msgs::LaserScan >(nh_, scan_topic1_, 1);
  scan_sub2_ = new message_filters::Subscriber< sensor_msgs::LaserScan >(nh_, scan_topic2_, 1);
  scan_sub_ = new message_filters::Subscriber< sensor_msgs::LaserScan >(nh_, scan_topic_single_, 1);
  odom_sub_ = new message_filters::Subscriber< nav_msgs::Odometry >(nh_, "/odom", 1);

  /***/
  syncA1_ = new message_filters::Synchronizer< MySyncPolicyA >(MySyncPolicyA(10), *scan_sub1_,
                                                               *odom_sub_); //这里的10就是误差容忍数
  message_filters::Connection dis_connectionA1 =
      syncA1_->registerCallback(boost::bind(&AicExtraction::timeSynchronizerA1Callback, this, _1, _2));

  /***/
  syncA2_ = new message_filters::Synchronizer< MySyncPolicyA >(MySyncPolicyA(10), *scan_sub2_,
                                                               *odom_sub_); //这里的10就是误差容忍数
  message_filters::Connection dis_connectionA2 =
      syncA2_->registerCallback(boost::bind(&AicExtraction::timeSynchronizerA2Callback, this, _1, _2));

  /***/
  syncA_ = new message_filters::Synchronizer< MySyncPolicyA >(MySyncPolicyA(10), *scan_sub_,
                                                              *odom_sub_); //这里的10就是误差容忍数
  message_filters::Connection dis_connectionA =
      syncA_->registerCallback(boost::bind(&AicExtraction::timeSynchronizerACallback, this, _1, _2));

  if (multi_scan_)
    dis_connectionA.disconnect();
  else
  {
    dis_connectionA1.disconnect();
    dis_connectionA2.disconnect();
  }
}

void VShapeExtra::loadParam()
{
  nh_local_.param< string >("v_shape/scan_topic1", scan_topic1_, "scan");
  nh_local_.param< string >("v_shape/scan_frame1", scan_frame1_, "laser_link");
  nh_local_.param< string >("v_shape/scan_topic2", scan_topic2_, "scan");
  nh_local_.param< string >("v_shape/scan_frame2", scan_frame2_, "laser_link");
  nh_local_.param< string >("v_shape/scan_topic", scan_topic_single_, "scan");
  nh_local_.param< string >("v_shape/scan_frame", scan_frame_single_, "laser_link");
  nh_local_.param< double >("v_shape/max_lines_1_2_angle", max_lines_1_2_angle_, 0.3);
  nh_local_.param< double >("v_shape/min_lines_1_2_angle", min_lines_1_2_angle_, 0.3);
  nh_local_.param< double >("v_shape/max_lines_2_3_angle", max_lines_2_3_angle_, 0.3);
  nh_local_.param< double >("v_shape/min_lines_2_3_angle", min_lines_2_3_angle_, 0.3);
  nh_local_.param< double >("v_shape/max_lines_1_3_angle", max_lines_1_3_angle_, 0.3);
  nh_local_.param< double >("v_shape/min_lines_1_3_angle", min_lines_1_3_angle_, 0.3);
  nh_local_.param< double >("v_shape/max_lines1", max_lines1_, 0.3);
  nh_local_.param< double >("v_shape/min_lines1", min_lines1_, 0.3);
  nh_local_.param< double >("v_shape/max_lines2", max_lines2_, 0.3);
  nh_local_.param< double >("v_shape/min_lines2", min_lines2_, 0.3);
  nh_local_.param< double >("v_shape/max_lines3", max_lines3_, 0.3);
  nh_local_.param< double >("v_shape/min_lines3", min_lines3_, 0.3);
}
void VShapeExtra::initParam()
{
  try
  {
    listerner_.waitForTransform("base_footprint", scan_frame_, ros::Time(0), ros::Duration(10.0));
    listerner_.lookupTransform("base_footprint", scan_frame_, ros::Time(0), foot_laser_frame_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  /**********************/
  double roll, pitch, yaw;
  tf::Matrix3x3(foot_laser_frame_.getRotation()).getRPY(roll, pitch, yaw);
  lidar_position_inverse_ = (((int)roll == -3) || ((int)roll == 3)) ? true : false;

  laser_laserNav_frame_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  if (!lidar_position_inverse_)
  {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    laser_laserNav_frame_.setRotation(q);
  }
  else
  {
    tf::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    laser_laserNav_frame_.setRotation(q);
  }
}

bool VShapeExtra::run(const geometry_msgs::Pose& old_middle_point, geometry_msgs::Pose& new_middle_point)
{
  initParam();

  if (receive_synchron_msg_ && scan_msg_ != nullptr)
  {
    receive_synchron_msg_ = false;

    tf::Transform old_middle_frame;
    tf::poseMsgToTF(old_middle_point, old_middle_frame);
    //    br_.sendTransform(tf::StampedTransform(old_middle_frame, ros::Time::now(), "odom", "old_Middle_frame"));

    pubExtraRangeMarker();

    /*** 直线提取，组合匹配 ***/
    boost::unique_lock< boost::recursive_mutex > lock(mutex_);
    aic_laser_feature_line_extraction::LineSegmentList lines;
    line_extractor_->laserScanCallback(scan_msg_);
    lines = line_extractor_->run();
    lock.unlock();

    if (lines.line_segments.empty())
    {
      ROS_INFO_THROTTLE(1, "VShape line extra no lines");
      ROS_DEBUG_NAMED("docking", "VShape line extra no lines");
      return false;
    }

    /*** 直线预处理 ***/
    vector< recognize_extraction::extra_line > input_extra_lines;
    vector< unsigned int > line_index;
    if (!infoPreprocess(lines, input_extra_lines, line_index))
      return false;

    /*** 提取符合条件的直线组合 ***/
    vector< geometry_msgs::Pose > vec_middle_point, vec_legal_point;
    vector< tuple< size_t, size_t, size_t > > temp_line_index, temp_legal_point;
    if (!extraLines(input_extra_lines, line_index, vec_middle_point, temp_line_index))
    {
      ROS_INFO_THROTTLE(1, "VShape extra line failed");
      ROS_DEBUG_NAMED("docking", "VShape extra line failed");
      return false;
    }

    if (!legalVShape(vec_middle_point, vec_legal_point, temp_legal_point))
    {
      ROS_INFO_THROTTLE(1, "legal v shape faild");
      ROS_DEBUG_NAMED("docking", "legal v shape faild");
      return false;
    }

    /*** 转换坐标系 ***/
    if (vec_legal_point.size() == 1)
    {
      ROS_INFO_THROTTLE(1, "v shape extra one middle point   angle_vel_:%f", angle_vel_);
      ROS_DEBUG_NAMED("docking", "v shape extra one middle point   angle_vel_:%f", angle_vel_);

      tf::Quaternion q;
      q.setRPY(0.0, 0.0, charging_port_delta_angle_);
      tf::Transform temp_frame;
      temp_frame.setRotation(q);
      temp_frame.setOrigin(tf::Vector3(0.0, charging_port_delta_y_, 0.0));

      tf::Transform new_laserNav_middle_frame;
      new_middle_point = vec_legal_point.front();
      tf::poseMsgToTF(new_middle_point, new_laserNav_middle_frame);
      tf::Transform new_foot_middle_frame =
          foot_laser_frame_ * laser_laserNav_frame_ * new_laserNav_middle_frame * temp_frame;

      tf::Transform new_odom_middle_frame = odom_foot_frame_ * new_foot_middle_frame;

      br_.sendTransform(tf::StampedTransform(new_odom_middle_frame, ros::Time::now(), "odom", "new_Middle_frame"));
      tf::poseTFToMsg(new_odom_middle_frame, new_middle_point);

      double roll, pitch, yaw;
      tf::Matrix3x3(new_odom_middle_frame.getRotation()).getRPY(roll, pitch, yaw);

      //      ROS_ERROR("x:%f, y:%f, theta:%f", new_odom_middle_frame.getOrigin().getX(),
      //                new_odom_middle_frame.getOrigin().getY(), yaw);

      //      tf::Transform old_odom_middle_frame, temp_odom_middle_frame;
      //      tf::poseMsgToTF(old_middle_point, old_odom_middle_frame);
      //      tf::poseMsgToTF(vec_legal_point.front(), temp_odom_middle_frame);
      //      tf::Transform deltatime_middle_frame = old_odom_middle_frame.inverse() * temp_odom_middle_frame;

      //      tf::Quaternion q(0.0, 0.0, 0.0);
      //      tf::Transform new_temp_middle_frame(
      //          q, tf::Vector3(0.0, deltatime_middle_frame.getOrigin().getY() + charging_port_delta_y_, 0.0));

      //      tf::Transform new_odom_middle_frame = old_odom_middle_frame * new_temp_middle_frame;
      //      tf::poseTFToMsg(new_odom_middle_frame, new_middle_point);
      //      br_.sendTransform(tf::StampedTransform(new_odom_middle_frame, ros::Time::now(), "odom",
      //      "new_Middle_frame"));
      //      br_.sendTransform(tf::StampedTransform(temp_odom_middle_frame, ros::Time::now(), "odom",
      //      "temp_Middle_frame"));

      return true;
    }
    else if (vec_legal_point.size() == 0)
    {
      ROS_INFO_THROTTLE(1, "v shape extra no middle point");
      ROS_DEBUG_NAMED("docking", "v shape extra no middle point");
      return false;
    }
    else
    {
      ROS_INFO_THROTTLE(1, "v shape extra more than one middle point");
      ROS_DEBUG_NAMED("docking", "v shape extra more than one middle point");
      return false;
    }
  }
  else
    return false;
}

bool VShapeExtra::infoPreprocess(const aic_laser_feature_line_extraction::LineSegmentList& lines,
                                 vector< recognize_extraction::extra_line >& input_extra_lines,
                                 vector< unsigned int >& line_index)
{
  /*** 直线坐标系从原本的laser_frame转变为foot_，只提取在识别框内的特征 ***/
  aic_laser_feature_line_extraction::LineSegmentList detect_lines;
  if (!lidar_position_inverse_)
  {
    for (vector< aic_laser_feature_line_extraction::LineSegment >::const_iterator cit = lines.line_segments.begin();
         cit != lines.line_segments.end(); cit++)
    {
      tf::Quaternion q;
      q.setRPY(0, 0, cit->angle);
      tf::Transform laser_startPt_frame(q, tf::Vector3(cit->start.at(0), cit->start.at(1), 0));
      tf::Transform laserNav_startPt_frame = laser_laserNav_frame_.inverse() * laser_startPt_frame;
      tf::Transform foot_startPt_frame = foot_laser_frame_ * laser_startPt_frame;
      tf::Transform odom_startPt_frame = odom_foot_frame_ * foot_startPt_frame;

      tf::Transform laser_endPt_frame(q, tf::Vector3(cit->end.at(0), cit->end.at(1), 0));
      tf::Transform laserNav_endPt_frame = laser_laserNav_frame_.inverse() * laser_endPt_frame;
      tf::Transform foot_endPt_frame = foot_laser_frame_ * laser_endPt_frame;
      tf::Transform odom_endPt_frame = odom_foot_frame_ * foot_endPt_frame;

      geometry_msgs::Point32 start, end;
      if (polygon_.frame_id == "odom")
      {
        point32TFToMsg(odom_startPt_frame.getOrigin(), start);
        point32TFToMsg(odom_endPt_frame.getOrigin(), end);
      }
      else if (polygon_.frame_id == "base_footprint")
      {
        point32TFToMsg(foot_startPt_frame.getOrigin(), start);
        point32TFToMsg(foot_endPt_frame.getOrigin(), end);
      }
      geometry_msgs::Polygon test_lines;
      test_lines.points.push_back(start);
      test_lines.points.push_back(end);

      if (aicpoly::GetCrossPolygon(test_lines, polygon_.extra_polygon) ||
          aicpoly::lines_in_polygon(test_lines, polygon_.extra_polygon))
      {
        double roll, pitch, yaw;
        tf::Matrix3x3(laserNav_startPt_frame.getRotation()).getRPY(roll, pitch, yaw);

        aic_laser_feature_line_extraction::LineSegment detect_line;
        detect_line.start.at(0) = laserNav_startPt_frame.getOrigin().x();
        detect_line.start.at(1) = laserNav_startPt_frame.getOrigin().y();
        detect_line.end.at(0) = laserNav_endPt_frame.getOrigin().x();
        detect_line.end.at(1) = laserNav_endPt_frame.getOrigin().y();
        detect_line.angle = yaw;
        detect_lines.line_segments.push_back(detect_line);
      }
    }
  }
  else
  {
    for (vector< aic_laser_feature_line_extraction::LineSegment >::const_reverse_iterator cit =
             lines.line_segments.rbegin();
         cit != lines.line_segments.rend(); cit++)
    {
      tf::Quaternion q;
      q.setRPY(0, 0, cit->angle);
      tf::Transform laser_startPt_frame(q, tf::Vector3(cit->start.at(0), cit->start.at(1), 0));
      tf::Transform laserNav_startPt_frame = laser_laserNav_frame_.inverse() * laser_startPt_frame;
      tf::Transform foot_startPt_frame = foot_laser_frame_ * laser_startPt_frame;
      tf::Transform odom_startPt_frame = odom_foot_frame_ * foot_startPt_frame;

      tf::Transform laser_endPt_frame(q, tf::Vector3(cit->end.at(0), cit->end.at(1), 0));
      tf::Transform laserNav_endPt_frame = laser_laserNav_frame_.inverse() * laser_endPt_frame;
      tf::Transform foot_endPt_frame = foot_laser_frame_ * laser_endPt_frame;
      tf::Transform odom_endPt_frame = odom_foot_frame_ * foot_endPt_frame;

      geometry_msgs::Point32 start, end;
      if (polygon_.frame_id == "odom")
      {
        point32TFToMsg(odom_startPt_frame.getOrigin(), start);
        point32TFToMsg(odom_endPt_frame.getOrigin(), end);
      }
      else if (polygon_.frame_id == "base_footprint")
      {
        point32TFToMsg(foot_startPt_frame.getOrigin(), start);
        point32TFToMsg(foot_endPt_frame.getOrigin(), end);
      }
      geometry_msgs::Polygon test_lines;
      test_lines.points.push_back(start);
      test_lines.points.push_back(end);

      if (aicpoly::GetCrossPolygon(test_lines, polygon_.extra_polygon) ||
          aicpoly::lines_in_polygon(test_lines, polygon_.extra_polygon))
      {
        double roll, pitch, yaw;
        tf::Matrix3x3(laserNav_startPt_frame.getRotation()).getRPY(roll, pitch, yaw);

        aic_laser_feature_line_extraction::LineSegment detect_line;
        detect_line.start.at(0) = laserNav_endPt_frame.getOrigin().x();
        detect_line.start.at(1) = laserNav_endPt_frame.getOrigin().y();
        detect_line.end.at(0) = laserNav_startPt_frame.getOrigin().x();
        detect_line.end.at(1) = laserNav_startPt_frame.getOrigin().y();
        detect_line.angle = yaw;

        detect_lines.line_segments.push_back(detect_line);
      }
    }
  }

  if (detect_lines.line_segments.empty())
  {
    ROS_INFO_THROTTLE(1, "VShape line extra: no lines in the extra range");
    ROS_DEBUG_NAMED("docking", "VShape line extra: no lines in the extra range");
    return false;
  }

  /*** init extra lines and line indices ***/
  int num = 0;
  for (vector< aic_laser_feature_line_extraction::LineSegment >::iterator cit = detect_lines.line_segments.begin();
       cit != detect_lines.line_segments.end(); ++num, ++cit)
  {
    recognize_extraction::extra_line extrac;
    extrac.start_x = cit->start.at(0);
    extrac.start_y = cit->start.at(1);
    extrac.end_x = cit->end.at(0);
    extrac.end_y = cit->end.at(1);
    extrac.num = num;
    extrac.angle = cit->angle;
    input_extra_lines.push_back(extrac);

    line_index.push_back(num);
  }
  return true;
}

/***
 *      *
 *     * * j
 *    *   *
 * i *     **** k
 ***/
bool VShapeExtra::extraLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                             const vector< unsigned int >& line_index, vector< geometry_msgs::Pose >& vec_middle_point,
                             vector< tuple< size_t, size_t, size_t > >& temp_line_index)
{
  /*** 计算合法的长度和角度的平行线集合 ***/
  size_t i, j, k;
  for (i = 0, j = input_extra_lines.size() - 1, k = input_extra_lines.size() - 1; i < input_extra_lines.size();
       k = j, j = i++)
  {
    double len1 = sqrt(pow((input_extra_lines[k].end_x - input_extra_lines[k].start_x), 2) +
                       pow((input_extra_lines[k].end_y - input_extra_lines[k].start_y), 2));
    double len2 = sqrt(pow((input_extra_lines[j].end_x - input_extra_lines[j].start_x), 2) +
                       pow((input_extra_lines[j].end_y - input_extra_lines[j].start_y), 2));
    double len3 = sqrt(pow((input_extra_lines[i].end_x - input_extra_lines[i].start_x), 2) +
                       pow((input_extra_lines[i].end_y - input_extra_lines[i].start_y), 2));

    double k1 = (input_extra_lines[k].end_y - input_extra_lines[k].start_y) /
                (input_extra_lines[k].end_x - input_extra_lines[k].start_x);
    double k2 = (input_extra_lines[j].end_y - input_extra_lines[j].start_y) /
                (input_extra_lines[j].end_x - input_extra_lines[j].start_x);
    double k3 = (input_extra_lines[i].end_y - input_extra_lines[i].start_y) /
                (input_extra_lines[i].end_x - input_extra_lines[i].start_x);
    double angle12 = recognize_extraction::RadToDeg(atan(fabs((k2 - k1) / (1 + k1 * k2))));
    double angle23 = recognize_extraction::RadToDeg(atan(fabs((k3 - k2) / (1 + k2 * k3))));
    double angle13 = recognize_extraction::RadToDeg(atan(fabs((k3 - k1) / (1 + k1 * k3))));

    bool bool_len, bool_angle;
    if (roughRecgonize_mark_)
    {
      bool_len = (min_lines1_ - 0.05 < len1) && (len1 < max_lines1_ + 0.05) && (min_lines2_ - 0.05 < len2) &&
                 (len2 < max_lines2_ + 0.05) && (min_lines3_ - 0.05 < len3) && (len3 < max_lines3_ + 0.05);
      bool_angle = (min_lines_1_2_angle_ - 10 < angle12) && (angle12 < max_lines_1_2_angle_ + 10) &&
                   (min_lines_2_3_angle_ - 10 < angle23) && (angle23 < max_lines_2_3_angle_ + 10) &&
                   (min_lines_1_3_angle_ - 10 < angle13) && (angle13 < max_lines_1_3_angle_ + 10);
    }
    else
    {
      bool_len = (min_lines1_ < len1) && (len1 < max_lines1_) && (min_lines2_ < len2) && (len2 < max_lines2_) &&
                 (min_lines3_ < len3) && (len3 < max_lines3_);
      bool_angle = (min_lines_1_2_angle_ < angle12) && (angle12 < max_lines_1_2_angle_) &&
                   (min_lines_2_3_angle_ < angle23) && (angle23 < max_lines_2_3_angle_) &&
                   (min_lines_1_3_angle_ < angle13) && (angle13 < max_lines_1_3_angle_);
    }

    ROS_DEBUG_NAMED("docking", "bool_angle:%d, angle12:%f,angle23:%f ,angle13:%f", bool_angle, angle12, angle23,
                    angle13);
    ROS_DEBUG_NAMED("docking", "bool_len:%d, len1:%f, len2:%f, len3:%f", bool_len, len1, len2, len3);

    if (bool_len && bool_angle)
    {
      ROS_DEBUG_NAMED("docking", "legal angle12:%f,angle23:%f ,angle13:%f", angle12, angle23, angle13);
      ROS_DEBUG_NAMED("docking", "legal len1:%f, len2:%f, len3:%f", len1, len2, len3);

      vector< geometry_msgs::Pose2D > pts;
      if (!(recognize_extraction::getCross(input_extra_lines[k].start_x, input_extra_lines[k].start_y,
                                           input_extra_lines[k].end_x, input_extra_lines[k].end_y,
                                           input_extra_lines[j].start_x, input_extra_lines[j].start_y,
                                           input_extra_lines[j].end_x, input_extra_lines[j].end_y, pts) &&
            recognize_extraction::getCross(input_extra_lines[j].start_x, input_extra_lines[j].start_y,
                                           input_extra_lines[j].end_x, input_extra_lines[j].end_y,
                                           input_extra_lines[i].start_x, input_extra_lines[i].start_y,
                                           input_extra_lines[i].end_x, input_extra_lines[i].end_y, pts) &&
            recognize_extraction::getCross(input_extra_lines[i].start_x, input_extra_lines[i].start_y,
                                           input_extra_lines[i].end_x, input_extra_lines[i].end_y,
                                           input_extra_lines[k].start_x, input_extra_lines[k].start_y,
                                           input_extra_lines[k].end_x, input_extra_lines[k].end_y, pts)))
      {
        ROS_DEBUG_NAMED("docking", "feature matching actionlib server: Error when calculating straight slope");
        return false;
      }

      geometry_msgs::Pose2D pt12, pt23, pt13, pt_middle;
      pt12 = pts.at(0);
      pt23 = pts.at(1);
      pt13 = pts.at(2);
      recognize_extraction::middle_pt(pt12, pt13, pt_middle);
      recognize_extraction::middle_pt(pt_middle, pt23, pt_middle);
      //      recognize_extraction::middle_pt(pt12, pt23, pt_middle);

      vector< double > angles;

      //      angles.push_back(input_extra_lines[i].angle);
      //      angles.push_back(input_extra_lines[j].angle);
      //      double a = recognize_extraction::getAverageAngle(angles);
      //      angles.clear();
      //      angles.push_back(a);
      //      angles.push_back(input_extra_lines[k].angle);
      //      pt_middle.theta = recognize_extraction::getAverageAngle(angles);

      //      angles.push_back(input_extra_lines[i].angle);
      //      angles.push_back(input_extra_lines[k].angle);
      //      pt_middle.theta = recognize_extraction::getAverageAngle(angles);

      pt_middle.theta = input_extra_lines[k].angle;

      tf::Quaternion q(0, 0, pt_middle.theta + M_PI);
      tf::Transform laserNav_middle_frame(q, tf::Vector3(pt_middle.x, pt_middle.y, 0.0));

      q.setRPY(0, 0, 0);
      //      tf::Transform middle_port_frame(
      //          q, tf::Vector3(0.0385, 0.135 - pow(laserNav_middle_frame.getOrigin().getX(), 2) * 0.01, 0.0));
      tf::Transform middle_port_frame(q, tf::Vector3(0.0385, 0.135, 0.0));
      //      tf::Transform middle_port_frame(q, tf::Vector3(0.0385, 0.135 / 2, 0.0));
      //      tf::Transform middle_port_frame(q, tf::Vector3(0.0385, -0.1, 0.0));

      tf::Transform laserNav_port_frame = laserNav_middle_frame * middle_port_frame;
      br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_port_frame, ros::Time::now(), scan_frame_,
                                             "laser_portnav_frame"));
      br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_middle_frame, ros::Time::now(),
                                             scan_frame_, "laser_port_frame"));

      /*** 雷达延时补偿 ***/
      //      recognize_extraction::Point start = {0.0, 0.0};
      //      recognize_extraction::Point end = {laserNav_port_frame.getOrigin().x(),
      //      laserNav_port_frame.getOrigin().y()};
      //      recognize_extraction::Point pt1 = {0, 0};
      //      recognize_extraction::Point pt2 = {1, 0};
      //      double angle = recognize_extraction::getAngelOfXaxisM_PI(start, end, pt1, pt2);
      //      double dt = angle_vel_ * dtScanTime_;
      //      //      ROS_WARN("dt:%f, angle_vel:%f, time:%f", dt, angle_vel_, dtScanTime_);
      //      dt = angles::normalize_angle(dt);
      //      angle += dt;
      //      angle = angles::normalize_angle(angle);
      //      double dist = recognize_extraction::distance(start, end);
      //      double x = dist * cos(angle);
      //      double y = dist * sin(angle);
      //      laserNav_port_frame.setOrigin(tf::Vector3(x, y, laserNav_port_frame.getOrigin().z()));

      //      tf::Quaternion qw(0.0, 0.0, angles::normalize_angle(dt));
      //      tf::Transform a_temp(qw, tf::Vector3(0, 0, 0));
      //      laserNav_port_frame *= a_temp;

      /*** tf发布 ***/
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(pt12.x, pt12.y, 0.0));
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br_.sendTransform(
          tf::StampedTransform(laser_laserNav_frame_ * transform, ros::Time::now(), scan_frame_, "pt12_Marker"));

      transform.setOrigin(tf::Vector3(pt23.x, pt23.y, 0.0));
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br_.sendTransform(
          tf::StampedTransform(laser_laserNav_frame_ * transform, ros::Time::now(), scan_frame_, "pt23_Marker"));

      transform.setOrigin(tf::Vector3(pt13.x, pt13.y, 0.0));
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br_.sendTransform(
          tf::StampedTransform(laser_laserNav_frame_ * transform, ros::Time::now(), scan_frame_, "pt13_Marker"));

      geometry_msgs::Pose middle_point;
      tf::poseTFToMsg(laserNav_port_frame, middle_point);
      vec_middle_point.push_back(middle_point);

      tuple< size_t, size_t, size_t > length_legal_index;
      length_legal_index = std::make_tuple(i, j, k);
      temp_line_index.push_back(length_legal_index);
    }
  }

  if (temp_line_index.empty())
  {
    ROS_INFO_THROTTLE(1, "v shape extra: Lack of appropriate length and angle");
    ROS_DEBUG_NAMED("docking", "v shape extra: Lack of appropriate length and angle");
    return false;
  }

  return true;
}

bool VShapeExtra::legalVShape(const vector< geometry_msgs::Pose >& input_extra_port,
                              vector< geometry_msgs::Pose >& vec_middle_point,
                              vector< tuple< size_t, size_t, size_t > >& temp_line_index)
{
  for (int i = 0; i < input_extra_port.size(); i++)
  {
    tf::Transform laser_port_frame, laserNav_port_frame;
    tf::poseMsgToTF(input_extra_port[i], laserNav_port_frame);
    laser_port_frame = laser_laserNav_frame_ * laserNav_port_frame;

    double dt_x = 0.05;
    /***
     *    a1*****************a2
     *    *        *          *
     *    *       * * j       *
     *    *      *   *        *
     *    *   i *     **** k  *
     *    *        g          *
     *    a4*****************a3
     ***/
    tf::Transform port_k1, port_k2, port_k3, port_k4;
    port_k1.setOrigin(tf::Vector3(-dt_x, 0.03, 0.0));
    port_k2.setOrigin(tf::Vector3(-dt_x, 0.33, 0.0));
    port_k3.setOrigin(tf::Vector3(dt_x, 0.33, 0.0));
    port_k4.setOrigin(tf::Vector3(dt_x, 0.03, 0.0));

    geometry_msgs::Point32 k1, k2, k3, k4;
    k1.x = (laser_port_frame * port_k1).getOrigin().x();
    k1.y = (laser_port_frame * port_k1).getOrigin().y();
    k2.x = (laser_port_frame * port_k2).getOrigin().x();
    k2.y = (laser_port_frame * port_k2).getOrigin().y();
    k3.x = (laser_port_frame * port_k3).getOrigin().x();
    k3.y = (laser_port_frame * port_k3).getOrigin().y();
    k4.x = (laser_port_frame * port_k4).getOrigin().x();
    k4.y = (laser_port_frame * port_k4).getOrigin().y();
    geometry_msgs::Point32 array_k[] = {k1, k2, k3, k4};
    geometry_msgs::Polygon polygon_k;
    polygon_k.points.insert(polygon_k.points.begin(), std::begin(array_k), std::end(array_k));
    //    ROS_ERROR("plygon k size:%d", polygon_k.points.size());

    /*******/
    tf::Transform port_j1, port_j2, port_j3, port_j4, port_j;
    tf::Quaternion qj(0.0, 0.0, -M_PI / 6);
    port_j.setRotation(qj);
    port_j.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    port_j1.setOrigin(tf::Vector3(-dt_x, -0.14, 0.0));
    port_j2.setOrigin(tf::Vector3(-dt_x, -0.02, 0.0));
    port_j3.setOrigin(tf::Vector3(dt_x, -0.02, 0.0));
    port_j4.setOrigin(tf::Vector3(dt_x, -0.14, 0.0));

    geometry_msgs::Point32 j1, j2, j3, j4;
    j1.x = (laser_port_frame * port_j * port_j1).getOrigin().x();
    j1.y = (laser_port_frame * port_j * port_j1).getOrigin().y();
    j2.x = (laser_port_frame * port_j * port_j2).getOrigin().x();
    j2.y = (laser_port_frame * port_j * port_j2).getOrigin().y();
    j3.x = (laser_port_frame * port_j * port_j3).getOrigin().x();
    j3.y = (laser_port_frame * port_j * port_j3).getOrigin().y();
    j4.x = (laser_port_frame * port_j * port_j4).getOrigin().x();
    j4.y = (laser_port_frame * port_j * port_j4).getOrigin().y();
    geometry_msgs::Point32 array_j[] = {j1, j2, j3, j4};
    geometry_msgs::Polygon polygon_j;
    polygon_j.points.insert(polygon_j.points.begin(), std::begin(array_j), std::end(array_j));
    //    ROS_ERROR("plygon j size:%d", polygon_j.points.size());

    /*******/
    tf::Transform port_i1, port_i2, port_i3, port_i4, port_i;
    tf::Quaternion qi(0.0, 0.0, M_PI / 6);
    port_i.setRotation(qi);
    port_i.setOrigin(tf::Vector3(0.0, -0.27, 0.0));
    port_i1.setOrigin(tf::Vector3(-dt_x, 0.12, 0.0));
    port_i2.setOrigin(tf::Vector3(-dt_x, 0.0, 0.0));
    port_i3.setOrigin(tf::Vector3(dt_x, 0.0, 0.0));
    port_i4.setOrigin(tf::Vector3(dt_x, 0.12, 0.0));

    geometry_msgs::Point32 i1, i2, i3, i4;
    i1.x = (laser_port_frame * port_i * port_i1).getOrigin().x();
    i1.y = (laser_port_frame * port_i * port_i1).getOrigin().y();
    i2.x = (laser_port_frame * port_i * port_i2).getOrigin().x();
    i2.y = (laser_port_frame * port_i * port_i2).getOrigin().y();
    i3.x = (laser_port_frame * port_i * port_i3).getOrigin().x();
    i3.y = (laser_port_frame * port_i * port_i3).getOrigin().y();
    i4.x = (laser_port_frame * port_i * port_i4).getOrigin().x();
    i4.y = (laser_port_frame * port_i * port_i4).getOrigin().y();
    geometry_msgs::Point32 array_i[] = {i1, i2, i3, i4};
    geometry_msgs::Polygon polygon_i;
    polygon_i.points.insert(polygon_i.points.begin(), std::begin(array_i), std::end(array_i));
    //    ROS_ERROR("plygon i size:%d", polygon_i.points.size());

    /*******/
    tf::Transform port_g1, port_g2, port_g3, port_g4;
    port_g1.setOrigin(tf::Vector3(-dt_x, -0.03, 0.0));
    port_g2.setOrigin(tf::Vector3(-dt_x, -0.35, 0.0));
    port_g3.setOrigin(tf::Vector3(dt_x, -0.35, 0.0));
    port_g4.setOrigin(tf::Vector3(dt_x, -0.03, 0.0));

    geometry_msgs::Point32 g1, g2, g3, g4;
    g1.x = (laser_port_frame * port_g1).getOrigin().x();
    g1.y = (laser_port_frame * port_g1).getOrigin().y();
    g2.x = (laser_port_frame * port_g2).getOrigin().x();
    g2.y = (laser_port_frame * port_g2).getOrigin().y();
    g3.x = (laser_port_frame * port_g3).getOrigin().x();
    g3.y = (laser_port_frame * port_g3).getOrigin().y();
    g4.x = (laser_port_frame * port_g4).getOrigin().x();
    g4.y = (laser_port_frame * port_g4).getOrigin().y();
    geometry_msgs::Point32 array_g[] = {g1, g2, g3, g4};
    geometry_msgs::Polygon polygon_g;
    polygon_g.points.insert(polygon_g.points.begin(), std::begin(array_g), std::end(array_g));
    //    ROS_ERROR("plygon g size:%d", polygon_g.points.size());

    pubVShapeRectangle(polygon_k, 1);
    pubVShapeRectangle(polygon_j, 2);
    pubVShapeRectangle(polygon_i, 3);
    pubVShapeRectangle(polygon_g, 4);

    sensor_msgs::LaserScan scan_msg_k = *scan_msg_, scan_msg_j = *scan_msg_, scan_msg_i = *scan_msg_;

    double min = 0.0, max = 0.0;
    for (int i = 0; i < scan_msg_->ranges.size(); i++)
    {
      double laser_num_ = scan_msg_->ranges.size();
      double dist_laser = scan_msg_->ranges[i];
      double rot_offset = (laser_num_ * scan_msg_->angle_increment) / 2;
      double angle_laser = scan_msg_->angle_increment * i - rot_offset;
      double x = dist_laser * cos(angle_laser);
      double y = dist_laser * sin(angle_laser);

      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      tf::Transform laser_pt_frame(q, tf::Vector3(x, y, 0));
      geometry_msgs::Point32 xy;
      xy.x = laser_pt_frame.getOrigin().x();
      xy.y = laser_pt_frame.getOrigin().y();

      if (isnan(xy.x) || isnan(xy.y) || isinf(xy.x) || isinf(xy.y))
        continue;

      if (!aicpoly::point_in_polygon(xy, polygon_k))
      {
        scan_msg_k.ranges[i] = 0.0f / 0.0f;
      }

      if (!aicpoly::point_in_polygon(xy, polygon_j))
      {
        scan_msg_j.ranges[i] = 0.0f / 0.0f;
      }

      if (!aicpoly::point_in_polygon(xy, polygon_i))
      {
        scan_msg_i.ranges[i] = 0.0f / 0.0f;
      }

      if (aicpoly::point_in_polygon(xy, polygon_k))
      {
        tf::Transform port_pt_frame = laser_port_frame.inverse() * laser_pt_frame;
        if (port_pt_frame.getOrigin().getY() > max)
          max = port_pt_frame.getOrigin().getY();
      }
      if (aicpoly::point_in_polygon(xy, polygon_g))
      {
        tf::Transform port_pt_frame = laser_port_frame.inverse() * laser_pt_frame;
        if (port_pt_frame.getOrigin().getY() < min)
          min = port_pt_frame.getOrigin().getY();
      }
    }
    //    ROS_WARN("MAX:%f, MIN:%f", max, min);
    tf::Quaternion q(0.0, 0.0, 0.0);
    tf::Transform port_portNav_frame(q, tf::Vector3(0.0, (max + min) / 2, 0.0));
    laserNav_port_frame = laser_laserNav_frame_.inverse() * laser_port_frame * port_portNav_frame;

    /*** 直线提取，组合匹配 ***/
    //    aic_laser_feature_line_extraction::LineSegmentList lines_k, lines_j, lines_i, lines;
    //    line_extractor_->laserScanCallback(scan_msg_k);
    //    lines_k = line_extractor_->run();
    //    line_extractor_->laserScanCallback(scan_msg_j);
    //    lines_j = line_extractor_->run();
    //    line_extractor_->laserScanCallback(scan_msg_i);
    //    lines_i = line_extractor_->run();

    //    visualization_msgs::Marker marker_msg;
    //    populateMarkerMsg(lines_k, marker_msg, 1);
    //    populateMarkerMsg(lines_j, marker_msg, 2);
    //    populateMarkerMsg(lines_i, marker_msg, 3);
    //    marker_publisher_.publish(marker_msg);

    //    lines.line_segments.insert(lines.line_segments.end(), lines_i.line_segments.begin(),
    //    lines_i.line_segments.end());
    //    lines.line_segments.insert(lines.line_segments.end(), lines_j.line_segments.begin(),
    //    lines_j.line_segments.end());
    //    lines.line_segments.insert(lines.line_segments.end(), lines_k.line_segments.begin(),
    //    lines_k.line_segments.end());

    /*** 直线预处理 ***/
    //    vector< recognize_extraction::extra_line > input_extra_lines;
    //    vector< unsigned int > line_index;
    //    if (!infoPreprocess(lines, input_extra_lines, line_index))
    //      return false;

    //    vector<double> angles;
    //    double theta;
    //    angles.push_back(input_extra_lines[0].angle);
    //    angles.push_back(input_extra_lines[1].angle);
    //    double a = recognize_extraction::getAverageAngle(angles);
    //    angles.clear();
    //    angles.push_back(a);
    //    angles.push_back(input_extra_lines[2].angle);
    //    theta = recognize_extraction::getAverageAngle(angles);

    //    theta = input_extra_lines[0].angle;
    //    tf::Quaternion q(0.0, 0.0, theta + M_PI);
    //    laserNav_port_frame.setRotation(q);

    geometry_msgs::Pose temp_pose;
    tf::poseTFToMsg(laserNav_port_frame, temp_pose);
    vec_middle_point.push_back(temp_pose);

    //    /*** 提取符合条件的直线组合 ***/
    //    if (!extraLines(input_extra_lines, line_index, vec_middle_point, temp_line_index))
    //    {
    //      ROS_WARN_THROTTLE(1, "VShape extra line failed");
    //      RROS_DEBUG_NAMED("docking", "VShape extra line failed");

    //      return false;
    //    }
  }
  return true;
}

void VShapeExtra::point32TFToMsg(const tf::Point& tf_a, geometry_msgs::Point32& msg)
{
  msg.x = tf_a.x();
  msg.y = tf_a.y();
  msg.z = tf_a.z();
}

void VShapeExtra::pubExtraRangeMarker()
{
  visualization_msgs::Marker marker_msg;
  marker_msg.ns = "VShape extra range";
  marker_msg.id = 1;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.03;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 1.0;
  marker_msg.color.a = 1.0;

  size_t i, j;
  for (i = 0, j = polygon_.extra_polygon.points.size() - 1; i < polygon_.extra_polygon.points.size(); j = i++)
  {
    geometry_msgs::Point p_start;
    p_start.x = polygon_.extra_polygon.points[j].x;
    p_start.y = polygon_.extra_polygon.points[j].y;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = polygon_.extra_polygon.points[i].x;
    p_end.y = polygon_.extra_polygon.points[i].y;
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }

  marker_msg.header.frame_id = polygon_.frame_id;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.lifetime = ros::Duration(0.5);
  extra_range_marker_.publish(marker_msg);
}

void VShapeExtra::pubVShapeRectangle(const geometry_msgs::Polygon& points, int id)
{
  visualization_msgs::Marker marker_msg;
  marker_msg.ns = "VShape rectangle";
  marker_msg.id = id;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.01;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.2;
  marker_msg.color.b = 0.7;
  marker_msg.color.a = 1.0;

  size_t i, j;
  for (i = 0, j = points.points.size() - 1; i < points.points.size(); j = i++)
  {
    geometry_msgs::Point p_start;
    p_start.x = points.points[j].x;
    p_start.y = points.points[j].y;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = points.points[i].x;
    p_end.y = points.points[i].y;
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }

  marker_msg.header.frame_id = scan_frame_;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.lifetime = ros::Duration(0.5);
  vshape_rectangle_marker_.publish(marker_msg);
}

void VShapeExtra::populateMarkerMsg(const aic_laser_feature_line_extraction::LineSegmentList& lines,
                                    visualization_msgs::Marker& marker_msg, int id)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = id;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.03;
  marker_msg.color.r = 0.2;
  marker_msg.color.g = 0.4;
  marker_msg.color.b = 1.0;
  marker_msg.color.a = 1.0;
  for (vector< aic_laser_feature_line_extraction::LineSegment >::const_iterator cit = lines.line_segments.begin();
       cit != lines.line_segments.end(); cit++)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->start.at(0);
    p_start.y = cit->start.at(1);
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = cit->end.at(0);
    p_end.y = cit->end.at(1);
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = scan_frame_;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.lifetime = ros::Duration(0.5);
}

bool VShapeExtra::setInterface(const AicExtractionInterface& interface)
{
  interface_ = interface;
  setParam(interface_.tag, interface_.name);
}

bool VShapeExtra::setParam(std::string tag, std::string name)
{
  ROS_INFO("vshape extra tag no:%s, name:%s", tag.c_str(), name.c_str());

  vector< std::string > no_tag;
  vector< double > charging_port_delta_y, charging_port_delta_angle, delta_backDist;
  bool defaultParam = true;
  XmlRpc::XmlRpcValue behavior_list;
  if (nh_.getParam(name, behavior_list))
  {
    if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < behavior_list.size(); ++i)
      {
        if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          if (behavior_list[i].hasMember("no_tag") && behavior_list[i].hasMember("charging_port_delta_y") &&
              behavior_list[i].hasMember("charging_port_delta_angle") && behavior_list[i].hasMember("delta_backDist"))
          {
            defaultParam = false;
            no_tag.push_back(behavior_list[i]["no_tag"]);
            charging_port_delta_y.push_back(behavior_list[i]["charging_port_delta_y"]);
            charging_port_delta_angle.push_back(behavior_list[i]["charging_port_delta_angle"]);
            delta_backDist.push_back(behavior_list[i]["delta_backDist"]);
          }
          else
          {
            ROS_ERROR("yaml loader must have a name and a type and this does not. Using the default yaml loader "
                      "behaviors instead.");
            defaultParam = true;
            break;
          }
        }
        else
        {
          ROS_ERROR("yaml loader must be specified as maps, but they are XmlRpcType %d. We'll use the default "
                    "yaml loader instead.",
                    behavior_list[i].getType());
          defaultParam = true;
          break;
        }
      }
    }
    else
    {
      ROS_ERROR("The yaml loader specification must be a list, but is of XmlRpcType %d. We'll use the default "
                "yaml loader instead.",
                behavior_list.getType());
      defaultParam = true;
    }
  }
  else
  {
    ROS_ERROR("The yaml loader specification must have a specific name. We'll use the default "
              "yaml loader instead.");
    defaultParam = true;
  }

  if (defaultParam == false)
  {
    bool accept_targe = false;
    for (int i = 0; i < no_tag.size(); i++)
    {
      if (tag == no_tag.at(i))
      {
        ROS_INFO("Load yaml param: %s", tag.c_str());

        accept_targe = true;
        charging_port_delta_y_ = charging_port_delta_y.at(i);
        charging_port_delta_angle_ = charging_port_delta_angle.at(i);
        delta_backDist_ = delta_backDist.at(i);

        break;
      }
    }
    if (accept_targe == false)
    {
      ROS_ERROR("The accepted tag no is not include in the yaml loader specification. We'll use the default "
                "yaml loader instead.");
      defaultParam = true;
    }
  }

  if (defaultParam == true)
  {
    ROS_INFO("Default launch param");

    nh_local_.param< double >("VShape/charging_port_delta_y", charging_port_delta_y_, 0.0);
    nh_local_.param< double >("VShape/charging_port_delta_angle", charging_port_delta_angle_, 0.0);
    nh_local_.param< double >("VShape/delta_backDist", delta_backDist_, 0.0);

    return false;
  }

  return true;
}
