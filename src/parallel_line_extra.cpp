#include "aic_auto_dock/parallel_line_extra.h"

ParallelLineExtra::ParallelLineExtra(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : AicExtraction(nh, nh_local)
{
  extra_range_marker_ = nh_local_.advertise< visualization_msgs::Marker >("parallel/extra_range", 10);
  parallel_rectangle_marker_ = nh_local_.advertise< visualization_msgs::Marker >("parallel/rectangle", 10);

  loadParam();

  scan_sub1_ = new message_filters::Subscriber< sensor_msgs::LaserScan >(nh_, scan_topic1_, 10);
  scan_sub2_ = new message_filters::Subscriber< sensor_msgs::LaserScan >(nh_, scan_topic2_, 10);
  scan_sub_ = new message_filters::Subscriber< sensor_msgs::LaserScan >(nh_, scan_topic_single_, 10);
  odom_sub_ = new message_filters::Subscriber< nav_msgs::Odometry >(nh_, "/odom", 10);

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

void ParallelLineExtra::loadParam()
{
  nh_local_.param< string >("parallel/scan_topic1", scan_topic1_, "scan");
  nh_local_.param< string >("parallel/scan_frame1", scan_frame1_, "laser_link");
  nh_local_.param< string >("parallel/scan_topic2", scan_topic2_, "scan");
  nh_local_.param< string >("parallel/scan_frame2", scan_frame2_, "laser_link");
  nh_local_.param< string >("parallel/scan_topic", scan_topic_single_, "scan");
  nh_local_.param< string >("parallel/scan_frame", scan_frame_single_, "laser_link");
  nh_local_.param< double >("parallel/linesMinGap", lines_min_gap_, 0.3);
  nh_local_.param< double >("parallel/linesMaxGap", lines_max_gap_, 0.3);
  nh_local_.param< double >("parallel/maxmunLength", maxmun_length_, 0.3);
  nh_local_.param< double >("parallel/minimunLength", minimun_length_, 0.3);
  nh_local_.param< double >("parallel/minimunLine", minimun_line_, 0.3);
  nh_local_.param< double >("parallel/linesMinAngle", lines_min_angle_, 0.3);
  nh_local_.param< double >("parallel/linesMaxAngle", lines_max_angle_, 0.3);
}

void ParallelLineExtra::initParam()
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
}

bool ParallelLineExtra::run(const geometry_msgs::Pose& old_middle_point, geometry_msgs::Pose& new_middle_point)
{
  initParam();
  initLaserNav();

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
      ROS_INFO_THROTTLE(1, "parallel line extra no lines");
      ROS_DEBUG_NAMED("docking", "parallel line extra no lines");
      return false;
    }

    /*** 直线预处理 ***/
    vector< recognize_extraction::extra_line > input_extra_lines;
    vector< unsigned int > line_index;
    if (!infoPreprocess(lines, input_extra_lines, line_index))
      return false;

    /*** 提取符合条件的直线组合 ***/
    vector< geometry_msgs::Pose > vec_middle_point;
    vector< pair< int, int > > temp_line_index;
    vector< unsigned int > single_temp_line_index;

    if (!extraLines(input_extra_lines, line_index, vec_middle_point, temp_line_index))
    {
      ROS_INFO_THROTTLE(1, "parallel extra line failed");
      ROS_DEBUG_NAMED("docking", "parallel extra line failed");

      vec_middle_point.clear();
      temp_line_index.clear();

      //      vector< unsigned int > rough_temp_line_index;
      //      if (!extraRoughLines(input_extra_lines, line_index, vec_middle_point, rough_temp_line_index))
      //      {
      //        ROS_WARN_THROTTLE(1, "rough parallel extra line failed");
      //        ROS_DEBUG_NAMED("docking", "rough parallel extra line failed");
      //        return false;
      //      }
      if (!extraSingleLines(input_extra_lines, line_index, vec_middle_point, single_temp_line_index))
      {
        ROS_WARN_THROTTLE(1, "single parallel extra line failed");
        ROS_DEBUG_NAMED("docking", "single parallel extra line failed");
        return false;
      }
    }

    /*** 转换坐标系 ***/
    if (vec_middle_point.size() == 1)
    {
      ROS_INFO_THROTTLE(1, "parallel extra one middle point");
      ROS_DEBUG_NAMED("docking", "parallel extra one middle point");

      bool bool_len, bool_len2;
      if (temp_line_index.size() == 1) // double line
      {
        ROS_DEBUG_NAMED("docking", "double line");
        input_extra_lines.at(temp_line_index.front().first);
        double len1 = sqrt(pow((input_extra_lines[temp_line_index.front().first].end_x -
                                input_extra_lines[temp_line_index.front().first].start_x),
                               2) +
                           pow((input_extra_lines[temp_line_index.front().first].end_y -
                                input_extra_lines[temp_line_index.front().first].start_y),
                               2));
        double len2 = sqrt(pow((input_extra_lines[temp_line_index.front().second].end_x -
                                input_extra_lines[temp_line_index.front().second].start_x),
                               2) +
                           pow((input_extra_lines[temp_line_index.front().second].end_y -
                                input_extra_lines[temp_line_index.front().second].start_y),
                               2));

        bool_len = (len1 > 0.25) && (len2 > 0.25);
        bool_len2 = false;
        // ((len1 > minimun_line_ && len1 < maxmun_length_) || (len2 > minimun_line_ && len2 < maxmun_length_));
        ROS_DEBUG_NAMED("docking", "length:len1:%f, len2:%f, minimun_line_:%f, maxmun_length_:%f", len1, len2,
                        minimun_line_, maxmun_length_);
      }
      else if (single_temp_line_index.size() == 1) // single line
      {
        ROS_DEBUG_NAMED("docking", "single line");
        bool_len = true;
        bool_len2 = false;
      }

      //      if (bool_len)
      //      {
      //              ROS_WARN("full line, charging_port_delta_y:%f, len1:%f, len2:%f", charging_port_delta_y_, len1,
      //              len2);

      //              tf::Quaternion q;
      //              q.setRPY(0.0, 0.0, charging_port_delta_angle_);
      //              tf::Transform temp_frame(
      //                  q, tf::Vector3(maxmun_length_ - fabs(charging_port_ + delta_backDist_),
      //                  charging_port_delta_y_,
      //                  0.0));

      //              tf::Transform temp_laserNav_middle_frame;
      //              tf::poseMsgToTF(vec_middle_point.front(), temp_laserNav_middle_frame);
      //              temp_laserNav_middle_frame *= temp_frame;
      //              tf::Transform new_foot_middle_frame = foot_laser_frame_ * laser_laserNav_frame_ *
      //              temp_laserNav_middle_frame;
      //              tf::Transform new_odom_middle_frame = odom_foot_frame_ * new_foot_middle_frame;

      //              br_.sendTransform(tf::StampedTransform(new_odom_middle_frame, ros::Time::now(), "odom",
      //              "new_Middle_frame"));
      //              tf::poseTFToMsg(new_odom_middle_frame, new_middle_point);
      //              return true;
      //      }
      //      else
      if (bool_len)
      {
        ROS_DEBUG_NAMED("docking", "only new middle pt");
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, charging_port_delta_angle_);
        tf::Transform temp_frame;
        temp_frame.setRotation(q);
        if (!far_detection_)
          temp_frame.setOrigin(
              tf::Vector3(maxmun_length_ - fabs(charging_port_ + delta_backDist_), charging_port_delta_y_, 0.0));
        else
          temp_frame.setOrigin(
              tf::Vector3(-fabs(charging_port_ + delta_backDist_), charging_port_delta_y_, 0.0));

        tf::Transform temp_laserNav_middle_frame;
        tf::poseMsgToTF(vec_middle_point.front(), temp_laserNav_middle_frame);
        temp_laserNav_middle_frame *= temp_frame;
        tf::Transform new_foot_middle_frame = foot_laser_frame_ * laser_laserNav_frame_ * temp_laserNav_middle_frame;
        tf::Transform new_odom_middle_frame = odom_foot_frame_ * new_foot_middle_frame;

        br_.sendTransform(
            tf::StampedTransform(new_odom_middle_frame, ros::Time::now(), "odom", "full_new_Middle_frame"));
        tf::poseTFToMsg(new_odom_middle_frame, new_middle_point);
        return true;
      }
      else if (bool_len2)
      {
        ROS_DEBUG_NAMED("docking", "merge double line and single line");
        tf::Transform old_odom_middle_frame;
        tf::poseMsgToTF(old_middle_point, old_odom_middle_frame);

        tf::Quaternion q;
        q.setRPY(0.0, 0.0, charging_port_delta_angle_);
        tf::Transform temp_frame(
            q, tf::Vector3(maxmun_length_ - fabs(charging_port_ + delta_backDist_), charging_port_delta_y_, 0.0));

        tf::Transform temp_laserNav_middle_frame;
        tf::poseMsgToTF(vec_middle_point.front(), temp_laserNav_middle_frame);
        temp_laserNav_middle_frame *= temp_frame;
        tf::Transform new_foot_middle_frame = foot_laser_frame_ * laser_laserNav_frame_ * temp_laserNav_middle_frame;
        tf::Transform new_odom_middle_frame = odom_foot_frame_ * new_foot_middle_frame;

        q.setRPY(0.0, 0.0, 0.0);
        tf::Transform deltatime_middle_frame(
            q, tf::Vector3((old_odom_middle_frame.inverse() * new_odom_middle_frame).getOrigin().getX(), 0.0, 0.0));
        if (deltatime_middle_frame.getOrigin().getX() >= 0)
          return false;

        new_odom_middle_frame = old_odom_middle_frame * deltatime_middle_frame;
        br_.sendTransform(
            tf::StampedTransform(new_odom_middle_frame, ros::Time::now(), "odom", "broken_new_Middle_frame"));
        tf::poseTFToMsg(new_odom_middle_frame, new_middle_point);
        return true;
      }
      //      {
      //        ROS_DEBUG_NAMED("docking", "broken line, charging_port_delta_y:%f", charging_port_delta_y_);

      //        tf::Transform old_odom_middle_frame, temp_laserNav_middle_frame;
      //        tf::poseMsgToTF(old_middle_point, old_odom_middle_frame);
      //        tf::poseMsgToTF(vec_middle_point.front(), temp_laserNav_middle_frame);
      //        tf::Transform new_foot_middle_frame = foot_laser_frame_ * laser_laserNav_frame_ *
      //        temp_laserNav_middle_frame;
      //        tf::Transform new_odom_middle_frame = odom_foot_frame_ * new_foot_middle_frame;
      //        tf::Transform deltatime_middle_frame = old_odom_middle_frame.inverse() * new_odom_middle_frame;

      //        double roll, pitch, yaw;
      //        tf::Matrix3x3(deltatime_middle_frame.getRotation()).getRPY(roll, pitch, yaw);

      //        tf::Quaternion q(0.0, 0.0, yaw + charging_port_delta_angle_);
      //        tf::Transform new_temp_middle_frame(
      //            q, tf::Vector3(deltatime_middle_frame.getOrigin().getX() + (maxmun_length_ - fabs(charging_port_ +
      //            delta_backDist_)),
      //                           deltatime_middle_frame.getOrigin().getY() + charging_port_delta_y_, 0.0));

      //        tf::Transform transform_odom_middle_frame = old_odom_middle_frame * new_temp_middle_frame;
      //        tf::poseTFToMsg(transform_odom_middle_frame, new_middle_point);
      //        br_.sendTransform(
      //            tf::StampedTransform(transform_odom_middle_frame, ros::Time::now(), "odom", "new_Middle_frame"));
      //        br_.sendTransform(tf::StampedTransform(new_odom_middle_frame, ros::Time::now(), "odom",
      //        "temp_Middle_frame"));
      //        return true;
      //      }
      return false;
    }
    else if (vec_middle_point.size() == 0)
    {
      ROS_INFO_THROTTLE(1, "parallel extra no middle point");
      ROS_DEBUG_NAMED("docking", "parallel extra no middle point");

      return false;
    }
    else
    {
      ROS_INFO_THROTTLE(1, "parallel extra more than one middle point");
      ROS_DEBUG_NAMED("docking", "parallel extra more than one middle point");
      return false;
    }
  }
  return false;
}

void ParallelLineExtra::initLaserNav()
{
  /**********************/
  double roll, pitch, yaw;
  tf::Matrix3x3(foot_laser_frame_.getRotation()).getRPY(roll, pitch, yaw);
  lidar_position_inverse_ = (((int)roll == -3) || ((int)roll == 3)) ? true : false;

  tf::Quaternion q;
  if (!lidar_position_inverse_)
    q.setRPY(0.0, 0.0, 0.0);
  else
    q.setRPY(M_PI, 0.0, 0.0);
  tf::Transform laser_laserTemp_frame(q, tf::Vector3(0.0, 0.0, 0.0));

  double yawTemp = yaw;
  if (dock_direction_ == docking_direction::forward)
  {
    yawTemp = -yaw;
  }
  else if (dock_direction_ == backward)
  {
    if (angles::normalize_angle(yaw) >= 0)
    {
      yawTemp = M_PI - yaw;
    }
    else if (angles::normalize_angle(yaw) < 0)
    {
      yawTemp = -(yaw + M_PI);
    }
  }
  q.setRPY(0.0, 0.0, yawTemp);
  tf::Transform laserTemp_laserNav_frame(q, tf::Vector3(0.0, 0.0, 0.0));

  tf::Transform laser_laserNav_frame = laser_laserTemp_frame * laserTemp_laserNav_frame;
  laser_laserNav_frame_.setData(laser_laserNav_frame);

  br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_, ros::Time::now(), scan_frame_, "laserNav_link"));
}

bool ParallelLineExtra::infoPreprocess(const aic_laser_feature_line_extraction::LineSegmentList& lines,
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
    ROS_INFO_THROTTLE(1, "parallel line extra: no lines in the extra range");
    ROS_DEBUG_NAMED("docking", "parallel line extra: no lines in the extra range");
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

bool ParallelLineExtra::extraLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                                   const vector< unsigned int >& line_index,
                                   vector< geometry_msgs::Pose >& vec_middle_point,
                                   vector< pair< int, int > >& temp_line_index)
{
  /*** 计算每段直线中点的集合 ***/
  vector< geometry_msgs::Pose2D > vec_pt_middle;
  for (vector< unsigned int >::const_iterator cit = line_index.begin(); cit != line_index.end(); ++cit)
  {
    geometry_msgs::Pose2D pt1, pt2, pt_middle;
    pt1.x = input_extra_lines[*cit].start_x;
    pt1.y = input_extra_lines[*cit].start_y;
    pt2.x = input_extra_lines[*cit].end_x;
    pt2.y = input_extra_lines[*cit].end_y;
    recognize_extraction::middle_pt(pt1, pt2, pt_middle);
    pt_middle.theta = input_extra_lines[*cit].angle;

    vec_pt_middle.push_back(pt_middle);
  }

  /*** 计算间距合法的直线组合 ***/
  vector< pair< int, int > > vec_gap_legal_index;
  for (int i = 0; i < input_extra_lines.size(); i++)
  {
    for (int j = i; j < input_extra_lines.size(); j++)
    {
      recognize_extraction::Point start_i = {input_extra_lines.at(i).start_x, input_extra_lines.at(i).start_y};
      recognize_extraction::Point end_i = {input_extra_lines.at(i).end_x, input_extra_lines.at(i).end_y};
      recognize_extraction::Point pt_j = {vec_pt_middle.at(j).x, vec_pt_middle.at(j).y};
      double len1 = recognize_extraction::distPtLine(start_i, end_i, pt_j);

      recognize_extraction::Point start_j = {input_extra_lines.at(j).start_x, input_extra_lines.at(j).start_y};
      recognize_extraction::Point end_j = {input_extra_lines.at(j).end_x, input_extra_lines.at(j).end_y};
      recognize_extraction::Point pt_i = {vec_pt_middle.at(i).x, vec_pt_middle.at(i).y};
      double len2 = recognize_extraction::distPtLine(start_j, end_j, pt_i);

      ROS_DEBUG_NAMED("docking", "gap1:%f, gap2:%f", len1, len2);

      if (len1 > lines_min_gap_ && len1 < lines_max_gap_ && len2 > lines_min_gap_ && len2 < lines_max_gap_)
      {
        ROS_DEBUG_NAMED("docking", "legal gap1:%f, gap2:%f", len1, len2);

        pair< int, int > legal_index;
        legal_index.first = line_index.at(i);
        legal_index.second = line_index.at(j);

        vec_gap_legal_index.push_back(legal_index);
      }
    }
  }

  if (vec_gap_legal_index.empty())
  {
    ROS_INFO_THROTTLE(1, "parallel line extra: there are't legal gap between lines");
    ROS_DEBUG_NAMED("docking", "parallel line extra: there are't legal gap between lines");
    return false;
  }

  /*** 计算合法的长度和角度的平行线集合 ***/
  vector< pair< int, int > > vec_length_legal_index;
  for (vector< pair< int, int > >::iterator cit = vec_gap_legal_index.begin(); cit != vec_gap_legal_index.end(); ++cit)
  {
    double len1 = sqrt(pow((input_extra_lines[(*cit).first].end_x - input_extra_lines[(*cit).first].start_x), 2) +
                       pow((input_extra_lines[(*cit).first].end_y - input_extra_lines[(*cit).first].start_y), 2));
    double len2 = sqrt(pow((input_extra_lines[(*cit).second].end_x - input_extra_lines[(*cit).second].start_x), 2) +
                       pow((input_extra_lines[(*cit).second].end_y - input_extra_lines[(*cit).second].start_y), 2));

    bool bool_len = (len1 > minimun_line_ && len1 < maxmun_length_ && len2 > minimun_line_ && len2 < maxmun_length_);

    double k1 = (input_extra_lines[(*cit).first].end_y - input_extra_lines[(*cit).first].start_y) /
                (input_extra_lines[(*cit).first].end_x - input_extra_lines[(*cit).first].start_x);
    double k2 = (input_extra_lines[(*cit).second].end_y - input_extra_lines[(*cit).second].start_y) /
                (input_extra_lines[(*cit).second].end_x - input_extra_lines[(*cit).second].start_x);
    double angle = atan(fabs((k2 - k1) / (1 + k1 * k2)));

    bool bool_angle = (angle < lines_max_angle_ && angle > lines_min_angle_);

    ROS_DEBUG_NAMED("docking", "bool_angle:%d, angle:%f, bool_len:%d, len1:%f, len2:%f", bool_angle, angle, bool_len,
                    len1, len2);

    if (bool_angle && bool_len)
    {
      ROS_DEBUG_NAMED("docking", "legal bool_angle:%d, angle:%f, bool_len:%d, len1:%f, len2:%f", bool_angle, angle,
                      bool_len, len1, len2);
      vec_length_legal_index.push_back(*cit);
    }
  }

  if (vec_length_legal_index.empty())
  {
    ROS_INFO_THROTTLE(1, "parallel line extra: Lack of appropriate length and angle");
    ROS_DEBUG_NAMED("docking", "parallel line extra: Lack of appropriate length and angle");
    return false;
  }

  /*** 坐标转换 ***/
  for (vector< pair< int, int > >::iterator cit = vec_length_legal_index.begin(); cit != vec_length_legal_index.end();
       ++cit)
  {
    geometry_msgs::Point32 Pt1start, Pt1end, Pt2start, Pt2end;
    Pt1start.x = input_extra_lines.at((*cit).first).start_x;
    Pt1start.y = input_extra_lines.at((*cit).first).start_y;

    Pt1end.x = input_extra_lines.at((*cit).first).end_x;
    Pt1end.y = input_extra_lines.at((*cit).first).end_y;

    Pt2start.x = input_extra_lines.at((*cit).second).start_x;
    Pt2start.y = input_extra_lines.at((*cit).second).start_y;

    Pt2end.x = input_extra_lines.at((*cit).second).end_x;
    Pt2end.y = input_extra_lines.at((*cit).second).end_y;

    geometry_msgs::Point32 pt1_choose, pt2_choose;
    if (!far_detection_)
    {
      pt1_choose = Pt1start.x >= Pt1end.x ? Pt1start : Pt1end;
      pt2_choose = Pt2start.x >= Pt2end.x ? Pt2start : Pt2end;
    }
    else
    {
      pt1_choose = Pt1start.x >= Pt1end.x ? Pt1end : Pt1start;
      pt2_choose = Pt2start.x >= Pt2end.x ? Pt2end : Pt2start;
    }

    //    int choose_cit = pt1_choose > pt2_choose ? (*cit).first : (*cit).second;

    tf::Transform laserNav_pt1_frame, laserNav_pt2_frame;
    tf::Quaternion q;

    laserNav_pt1_frame.setOrigin(tf::Vector3(pt1_choose.x, pt1_choose.y, 0.0));
    q.setRPY(0, 0, input_extra_lines.at(cit->first).angle + M_PI);
    laserNav_pt1_frame.setRotation(q);
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_pt1_frame, ros::Time::now(), scan_frame_, "pt1"));

    laserNav_pt2_frame.setOrigin(tf::Vector3(pt2_choose.x, pt2_choose.y, 0.0));
    q.setRPY(0, 0, input_extra_lines.at(cit->second).angle + M_PI);
    laserNav_pt2_frame.setRotation(q);
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_pt2_frame, ros::Time::now(), scan_frame_, "pt2"));

    /********************/
    tf::Transform pt1_pt2_frame = laserNav_pt1_frame.inverse() * laserNav_pt2_frame;

    double boards_gap = pt1_pt2_frame.getOrigin().getX() > 0 ? (lines_max_gap_ + lines_min_gap_) / 4
                                                             : -(lines_max_gap_ + lines_min_gap_) / 4;

    tf::Transform pt1_middle_frame;
    pt1_middle_frame.setOrigin(tf::Vector3(boards_gap, 0.0, 0.0));
    pt1_middle_frame.setRotation(q);

    //    double angle1 = (input_extra_lines.at(cit->first).angle + input_extra_lines.at(cit->second).angle) / 2 + M_PI;
    double angle1 = input_extra_lines.at(cit->first).angle > 0 ? (input_extra_lines.at(cit->first).angle + M_PI_2)
                                                               : (input_extra_lines.at(cit->first).angle - M_PI_2);
    tf::Transform laserNav_middle_frame;
    laserNav_middle_frame = laserNav_pt1_frame * pt1_middle_frame;
    q.setRPY(0, 0, angle1);
    laserNav_middle_frame.setRotation(q);

    tf::Transform temp1 = laserNav_middle_frame.inverse() * laserNav_pt1_frame;
    tf::Transform temp2 = laserNav_middle_frame.inverse() * laserNav_pt2_frame;
    double p;
    if (!far_detection_)
    {
      if (temp1.getOrigin().getX() > temp2.getOrigin().getX())
        p = temp2.getOrigin().getX();
    }
    else
    {
      if (temp1.getOrigin().getX() < temp2.getOrigin().getX())
        p = temp2.getOrigin().getX();
    }
    tf::Quaternion pq;
    pq.setRPY(0.0, 0.0, 0.0);
    tf::Transform pf(pq, tf::Vector3(p, 0.0, 0.0));
    laserNav_middle_frame *= pf;

    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_middle_frame, ros::Time::now(), scan_frame_, "pt"));

    geometry_msgs::Pose middle_point;
    tf::poseTFToMsg(laserNav_middle_frame, middle_point);
    vec_middle_point.push_back(middle_point);
  }

  temp_line_index = vec_length_legal_index;
  return true;
}

bool ParallelLineExtra::extraRoughLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                                        const vector< unsigned int >& line_index,
                                        vector< geometry_msgs::Pose >& vec_middle_point,
                                        vector< unsigned int >& temp_line_index)
{
  /*** 计算合法的长度和角度的平行线集合 ***/
  vector< unsigned int > vec_length_legal_index;
  for (vector< unsigned int >::const_iterator cit = line_index.begin(); cit != line_index.end(); ++cit)
  {
    double len = sqrt(pow((input_extra_lines[(*cit)].end_x - input_extra_lines[(*cit)].start_x), 2) +
                      pow((input_extra_lines[(*cit)].end_y - input_extra_lines[(*cit)].start_y), 2));

    bool bool_len = (len > 0.25 && len < maxmun_length_);

    ROS_DEBUG_NAMED("parallel", "rough bool_len:%d, len:%f", bool_len, len);

    if (bool_len)
    {
      ROS_DEBUG_NAMED("parallel", "rough legal bool_len:%d, len:%f", bool_len, len);

      vec_length_legal_index.push_back(*cit);
    }
  }

  if (vec_length_legal_index.size() != 1)
  {
    ROS_DEBUG_NAMED("parallel", "rough parallel line extra: Lack of appropriate length");
    return false;
  }

  double min_dist = INFINITY;
  for (vector< unsigned int >::iterator cit = vec_length_legal_index.begin(); cit != vec_length_legal_index.end();
       ++cit)
  {
    double boards_gap = (lines_max_gap_ + lines_min_gap_) / 2 + 0.1;

    tf::Quaternion laserNavEndQ;
    laserNavEndQ.setRPY(0, 0, input_extra_lines[(*cit)].angle);
    tf::Transform laserNav_End_Frame(
        laserNavEndQ, tf::Vector3(input_extra_lines[(*cit)].end_x, input_extra_lines[(*cit)].end_y, 0.0));

    tf::Quaternion end_endExpandQ;
    end_endExpandQ.setRPY(0, 0, 0);
    tf::Transform end_endExpandFrame(end_endExpandQ, tf::Vector3(-boards_gap, 0.0, 0.0));
    tf::Transform laserNav_endExpandFrame = laserNav_End_Frame * end_endExpandFrame;

    tf::Quaternion laserNavStartQ;
    laserNavStartQ.setRPY(0, 0, input_extra_lines[(*cit)].angle);
    tf::Transform laserNav_Start_Frame(
        laserNavStartQ, tf::Vector3(input_extra_lines[(*cit)].start_x, input_extra_lines[(*cit)].start_y, 0.0));

    tf::Quaternion start_StartExpandQ;
    start_StartExpandQ.setRPY(0, 0, 0);
    tf::Transform start_StartExpandFrame(start_StartExpandQ, tf::Vector3(-boards_gap, 0.0, 0.0));
    tf::Transform laserNav_StartExpandFrame = laserNav_Start_Frame * start_StartExpandFrame;

    geometry_msgs::Point32 k1, k2, k3, k4;
    k1.x = (laser_laserNav_frame_ * laserNav_End_Frame).getOrigin().x();
    k1.y = (laser_laserNav_frame_ * laserNav_End_Frame).getOrigin().y();
    k2.x = (laser_laserNav_frame_ * laserNav_endExpandFrame).getOrigin().x();
    k2.y = (laser_laserNav_frame_ * laserNav_endExpandFrame).getOrigin().y();
    k3.x = (laser_laserNav_frame_ * laserNav_StartExpandFrame).getOrigin().x();
    k3.y = (laser_laserNav_frame_ * laserNav_StartExpandFrame).getOrigin().y();
    k4.x = (laser_laserNav_frame_ * laserNav_Start_Frame).getOrigin().x();
    k4.y = (laser_laserNav_frame_ * laserNav_Start_Frame).getOrigin().y();
    geometry_msgs::Point32 array_k[] = {k1, k2, k3, k4};
    geometry_msgs::Polygon polygon_k;
    polygon_k.points.insert(polygon_k.points.begin(), std::begin(array_k), std::end(array_k));
    pubParallelRectangle(polygon_k, 1);

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

      if (aicpoly::point_in_polygon(xy, polygon_k))
      {
        recognize_extraction::Point start_i = {laserNav_Start_Frame.getOrigin().x(),
                                               laserNav_Start_Frame.getOrigin().y()};
        recognize_extraction::Point end_i = {laserNav_End_Frame.getOrigin().x(), laserNav_End_Frame.getOrigin().y()};
        recognize_extraction::Point pt_j = {xy.x, xy.y};
        double len = recognize_extraction::distPtLine(start_i, end_i, pt_j);
        if (len < lines_max_gap_ && len > lines_min_gap_)
        {
          if (len < min_dist)
            min_dist = len;
        }
      }
    }
  }

  if (isinf(min_dist))
  {
    ROS_DEBUG_NAMED("parallel", "rough parallel line extra: gap is inf");
    return false;
  }

  /*** 坐标转换 ***/
  for (vector< unsigned int >::iterator cit = vec_length_legal_index.begin(); cit != vec_length_legal_index.end();
       ++cit)
  {
    tf::Quaternion laserNavEndQ;
    laserNavEndQ.setRPY(0, 0, input_extra_lines[(*cit)].angle);
    tf::Transform laserNav_End_Frame(
        laserNavEndQ, tf::Vector3(input_extra_lines[(*cit)].end_x, input_extra_lines[(*cit)].end_y, 0.0));
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_End_Frame, ros::Time::now(), scan_frame_, "end"));

    tf::Quaternion end_endExpandQ;
    end_endExpandQ.setRPY(0, 0, 0);
    tf::Transform end_endExpandFrame(end_endExpandQ, tf::Vector3(-min_dist, 0.0, 0.0));
    tf::Transform laserNav_endExpandFrame = laserNav_End_Frame * end_endExpandFrame;
    br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_endExpandFrame, ros::Time::now(),
                                           scan_frame_, "endExpand"));

    tf::Quaternion laserNavStartQ;
    laserNavStartQ.setRPY(0, 0, input_extra_lines[(*cit)].angle);
    tf::Transform laserNav_Start_Frame(
        laserNavStartQ, tf::Vector3(input_extra_lines[(*cit)].start_x, input_extra_lines[(*cit)].start_y, 0.0));
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_Start_Frame, ros::Time::now(), scan_frame_, "start"));

    tf::Quaternion start_StartExpandQ;
    start_StartExpandQ.setRPY(0, 0, 0);
    tf::Transform start_StartExpandFrame(start_StartExpandQ, tf::Vector3(-min_dist, 0.0, 0.0));
    tf::Transform laserNav_StartExpandFrame = laserNav_Start_Frame * start_StartExpandFrame;
    br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_StartExpandFrame, ros::Time::now(),
                                           scan_frame_, "startExpand"));

    geometry_msgs::Point32 Pt1start, Pt1end, Pt2start, Pt2end;
    Pt1start.x = laserNav_Start_Frame.getOrigin().getX();
    Pt1start.y = laserNav_Start_Frame.getOrigin().getY();

    Pt1end.x = laserNav_End_Frame.getOrigin().getX();
    Pt1end.y = laserNav_End_Frame.getOrigin().getY();

    Pt2start.x = laserNav_StartExpandFrame.getOrigin().getX();
    Pt2start.y = laserNav_StartExpandFrame.getOrigin().getY();

    Pt2end.x = laserNav_endExpandFrame.getOrigin().getX();
    Pt2end.y = laserNav_endExpandFrame.getOrigin().getY();

    geometry_msgs::Point32 pt1_choose = Pt1start.x >= Pt1end.x ? Pt1start : Pt1end;
    geometry_msgs::Point32 pt2_choose = Pt2start.x >= Pt2end.x ? Pt2start : Pt2end;

    tf::Transform laserNav_pt1_frame, laserNav_pt2_frame;
    tf::Quaternion q;

    laserNav_pt1_frame.setOrigin(tf::Vector3(pt1_choose.x, pt1_choose.y, 0.0));
    q.setRPY(0, 0, input_extra_lines.at(*cit).angle + M_PI);
    laserNav_pt1_frame.setRotation(q);
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_pt1_frame, ros::Time::now(), scan_frame_, "rough_pt1"));

    laserNav_pt2_frame.setOrigin(tf::Vector3(pt2_choose.x, pt2_choose.y, 0.0));
    q.setRPY(0, 0, input_extra_lines.at(*cit).angle);
    laserNav_pt2_frame.setRotation(q);
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_pt2_frame, ros::Time::now(), scan_frame_, "rough_pt2"));

    /********************/
    tf::Transform pt1_pt2_frame = laserNav_pt1_frame.inverse() * laserNav_pt2_frame;

    tf::Transform pt1_middle_frame;
    pt1_middle_frame.setOrigin(tf::Vector3(pt1_pt2_frame.getOrigin().getX() / 2, 0.0, 0.0));
    pt1_middle_frame.setRotation(q);

    double angle1 = input_extra_lines.at(*cit).angle > 0 ? (input_extra_lines.at(*cit).angle + M_PI_2)
                                                         : (input_extra_lines.at(*cit).angle - M_PI_2);
    tf::Transform laserNav_middle_frame;
    laserNav_middle_frame = laserNav_pt1_frame * pt1_middle_frame;
    q.setRPY(0, 0, angle1);
    laserNav_middle_frame.setRotation(q);

    br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_middle_frame, ros::Time::now(), scan_frame_,
                                           "rough_pttt"));

    tf::Transform temp1 = laserNav_middle_frame.inverse() * laserNav_pt1_frame;
    tf::Transform temp2 = laserNav_middle_frame.inverse() * laserNav_pt2_frame;
    double p;
    if (temp1.getOrigin().getX() > temp2.getOrigin().getX())
      p = temp2.getOrigin().getX();
    tf::Quaternion pq;
    pq.setRPY(0.0, 0.0, 0.0);
    tf::Transform pf(pq, tf::Vector3(p, 0.0, 0.0));
    tf::Transform laserNav_middle1_frame = laserNav_middle_frame * pf;

    br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_middle1_frame, ros::Time::now(),
                                           scan_frame_, "rough_pt"));

    geometry_msgs::Pose middle_point;
    tf::poseTFToMsg(laserNav_middle1_frame, middle_point);
    vec_middle_point.push_back(middle_point);
  }

  temp_line_index = vec_length_legal_index;
  return true;
}

bool ParallelLineExtra::extraSingleLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                                         const vector< unsigned int >& line_index,
                                         vector< geometry_msgs::Pose >& vec_middle_point,
                                         vector< unsigned int >& temp_line_index)
{
  /*** 计算合法的长度和角度的平行线集合 ***/
  vector< unsigned int > vec_length_legal_index;
  for (vector< unsigned int >::const_iterator cit = line_index.begin(); cit != line_index.end(); ++cit)
  {
    double len = sqrt(pow((input_extra_lines[(*cit)].end_x - input_extra_lines[(*cit)].start_x), 2) +
                      pow((input_extra_lines[(*cit)].end_y - input_extra_lines[(*cit)].start_y), 2));

    bool bool_len = (len > minimun_length_ && len < maxmun_length_);

    ROS_DEBUG_NAMED("docking", "single bool_len:%d, len:%f", bool_len, len);

    if (bool_len)
    {
      ROS_DEBUG_NAMED("docking", "single legal bool_len:%d, len:%f", bool_len, len);

      vec_length_legal_index.push_back(*cit);
    }
  }

  if (vec_length_legal_index.size() != 1)
  {
    ROS_DEBUG_NAMED("docking", "single parallel line extra: Lack of appropriate length");
    return false;
  }

  /*** 坐标转换 ***/
  for (vector< unsigned int >::iterator cit = vec_length_legal_index.begin(); cit != vec_length_legal_index.end();
       ++cit)
  {
    double boards_gap = (lines_max_gap_ + lines_min_gap_) / 2;

    tf::Quaternion laserNavEndQ;
    laserNavEndQ.setRPY(0, 0, input_extra_lines[(*cit)].angle);
    tf::Transform laserNav_End_Frame(
        laserNavEndQ, tf::Vector3(input_extra_lines[(*cit)].end_x, input_extra_lines[(*cit)].end_y, 0.0));
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_End_Frame, ros::Time::now(), scan_frame_, "end"));

    tf::Quaternion end_endExpandQ;
    end_endExpandQ.setRPY(0, 0, 0);
    tf::Transform end_endExpandFrame(end_endExpandQ, tf::Vector3(-boards_gap, 0.0, 0.0));
    tf::Transform laserNav_endExpandFrame = laserNav_End_Frame * end_endExpandFrame;
    br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_endExpandFrame, ros::Time::now(),
                                           scan_frame_, "endExpand"));

    tf::Quaternion laserNavStartQ;
    laserNavStartQ.setRPY(0, 0, input_extra_lines[(*cit)].angle);
    tf::Transform laserNav_Start_Frame(
        laserNavStartQ, tf::Vector3(input_extra_lines[(*cit)].start_x, input_extra_lines[(*cit)].start_y, 0.0));
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_Start_Frame, ros::Time::now(), scan_frame_, "start"));

    tf::Quaternion start_StartExpandQ;
    start_StartExpandQ.setRPY(0, 0, 0);
    tf::Transform start_StartExpandFrame(start_StartExpandQ, tf::Vector3(-boards_gap, 0.0, 0.0));
    tf::Transform laserNav_StartExpandFrame = laserNav_Start_Frame * start_StartExpandFrame;
    br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_StartExpandFrame, ros::Time::now(),
                                           scan_frame_, "startExpand"));

    geometry_msgs::Point32 Pt1start, Pt1end, Pt2start, Pt2end;
    Pt1start.x = laserNav_Start_Frame.getOrigin().getX();
    Pt1start.y = laserNav_Start_Frame.getOrigin().getY();

    Pt1end.x = laserNav_End_Frame.getOrigin().getX();
    Pt1end.y = laserNav_End_Frame.getOrigin().getY();

    Pt2start.x = laserNav_StartExpandFrame.getOrigin().getX();
    Pt2start.y = laserNav_StartExpandFrame.getOrigin().getY();

    Pt2end.x = laserNav_endExpandFrame.getOrigin().getX();
    Pt2end.y = laserNav_endExpandFrame.getOrigin().getY();

    geometry_msgs::Point32 pt1_choose, pt2_choose;
    if (!far_detection_)
    {
      pt1_choose = Pt1start.x >= Pt1end.x ? Pt1start : Pt1end;
      pt2_choose = Pt2start.x >= Pt2end.x ? Pt2start : Pt2end;
    }
    else
    {
      pt1_choose = Pt1start.x >= Pt1end.x ? Pt1end : Pt1start;
      pt2_choose = Pt2start.x >= Pt2end.x ? Pt2end : Pt2start;
    }

    tf::Transform laserNav_pt1_frame, laserNav_pt2_frame;
    tf::Quaternion q;

    laserNav_pt1_frame.setOrigin(tf::Vector3(pt1_choose.x, pt1_choose.y, 0.0));
    q.setRPY(0, 0, input_extra_lines.at(*cit).angle + M_PI);
    laserNav_pt1_frame.setRotation(q);
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_pt1_frame, ros::Time::now(), scan_frame_, "single_pt1"));

    laserNav_pt2_frame.setOrigin(tf::Vector3(pt2_choose.x, pt2_choose.y, 0.0));
    q.setRPY(0, 0, input_extra_lines.at(*cit).angle);
    laserNav_pt2_frame.setRotation(q);
    br_.sendTransform(
        tf::StampedTransform(laser_laserNav_frame_ * laserNav_pt2_frame, ros::Time::now(), scan_frame_, "single_pt2"));

    /********************/
    tf::Transform pt1_pt2_frame = laserNav_pt1_frame.inverse() * laserNav_pt2_frame;

    tf::Transform pt1_middle_frame;
    pt1_middle_frame.setOrigin(tf::Vector3(pt1_pt2_frame.getOrigin().getX() / 2, 0.0, 0.0));
    pt1_middle_frame.setRotation(q);

    double angle1 = input_extra_lines.at(*cit).angle > 0 ? (input_extra_lines.at(*cit).angle + M_PI_2)
                                                         : (input_extra_lines.at(*cit).angle - M_PI_2);
    tf::Transform laserNav_middle_frame;
    laserNav_middle_frame = laserNav_pt1_frame * pt1_middle_frame;
    q.setRPY(0, 0, angle1);
    laserNav_middle_frame.setRotation(q);

    br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_middle_frame, ros::Time::now(), scan_frame_,
                                           "single_pttt"));

    tf::Transform temp1 = laserNav_middle_frame.inverse() * laserNav_pt1_frame;
    tf::Transform temp2 = laserNav_middle_frame.inverse() * laserNav_pt2_frame;
    double p;
    if (!far_detection_)
    {
      if (temp1.getOrigin().getX() > temp2.getOrigin().getX())
        p = temp2.getOrigin().getX();
    }
    else
    {
      if (temp1.getOrigin().getX() < temp2.getOrigin().getX())
        p = temp2.getOrigin().getX();
    }
    tf::Quaternion pq;
    pq.setRPY(0.0, 0.0, 0.0);
    tf::Transform pf(pq, tf::Vector3(p, 0.0, 0.0));
    tf::Transform laserNav_middle1_frame = laserNav_middle_frame * pf;

    br_.sendTransform(tf::StampedTransform(laser_laserNav_frame_ * laserNav_middle1_frame, ros::Time::now(),
                                           scan_frame_, "single_pt"));

    geometry_msgs::Pose middle_point;
    tf::poseTFToMsg(laserNav_middle1_frame, middle_point);
    vec_middle_point.push_back(middle_point);
  }

  temp_line_index = vec_length_legal_index;
  return true;
}

void ParallelLineExtra::point32TFToMsg(const tf::Point& tf_a, geometry_msgs::Point32& msg)
{
  msg.x = tf_a.x();
  msg.y = tf_a.y();
  msg.z = tf_a.z();
}

void ParallelLineExtra::pubExtraRangeMarker()
{
  visualization_msgs::Marker marker_msg;
  marker_msg.ns = "parallel extra range";
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

void ParallelLineExtra::pubParallelRectangle(const geometry_msgs::Polygon& points, int id)
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
  parallel_rectangle_marker_.publish(marker_msg);
}

bool ParallelLineExtra::setInterface(const AicExtractionInterface& interface)
{
  interface_ = interface;
  setParam(interface_.tag, interface_.name);
}

bool ParallelLineExtra::setParam(std::string tag, std::string name)
{
  ROS_INFO("parallel line extra tag no:%s, name:%s", tag.c_str(), name.c_str());

  vector< std::string > no_tag;
  vector< double > lines_min_angle, lines_max_angle, lines_max_gap, lines_min_gap, maxmun_length, minimun_length,
      minimun_line, charging_port, charging_port_delta_y, charging_port_delta_angle, delta_backDist;
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
          if (behavior_list[i].hasMember("no_tag") && behavior_list[i].hasMember("min_line_angle") &&
              behavior_list[i].hasMember("max_line_angle") && behavior_list[i].hasMember("max_boards_gap") &&
              behavior_list[i].hasMember("min_boards_gap") && behavior_list[i].hasMember("maxmunLength") &&
              behavior_list[i].hasMember("minimunLength") && behavior_list[i].hasMember("minimunLine") &&
              behavior_list[i].hasMember("charging_port") && behavior_list[i].hasMember("charging_port_delta_y") &&
              behavior_list[i].hasMember("charging_port_delta_angle") && behavior_list[i].hasMember("delta_backDist"))
          {
            defaultParam = false;
            no_tag.push_back(behavior_list[i]["no_tag"]);
            lines_max_angle.push_back(behavior_list[i]["max_line_angle"]);
            lines_min_angle.push_back(behavior_list[i]["min_line_angle"]);
            lines_max_gap.push_back(behavior_list[i]["max_boards_gap"]);
            lines_min_gap.push_back(behavior_list[i]["min_boards_gap"]);
            maxmun_length.push_back(behavior_list[i]["maxmunLength"]);
            minimun_length.push_back(behavior_list[i]["minimunLength"]);
            minimun_line.push_back(behavior_list[i]["minimunLine"]);
            charging_port.push_back(behavior_list[i]["charging_port"]);
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
        lines_max_angle_ = lines_max_angle.at(i);
        lines_min_angle_ = lines_min_angle.at(i);
        lines_max_gap_ = lines_max_gap.at(i);
        lines_min_gap_ = lines_min_gap.at(i);
        maxmun_length_ = maxmun_length.at(i);
        minimun_length_ = minimun_length.at(i);
        minimun_line_ = minimun_line.at(i);
        charging_port_ = charging_port.at(i);
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

    nh_local_.param< double >("parallel/linesMinGap", lines_min_gap_, 0.3);
    nh_local_.param< double >("parallel/linesMaxGap", lines_max_gap_, 0.3);
    nh_local_.param< double >("parallel/maxmunLength", maxmun_length_, 0.3);
    nh_local_.param< double >("parallel/minimunLength", minimun_length_, 0.3);
    nh_local_.param< double >("parallel/minimunLine", minimun_line_, 0.3);
    nh_local_.param< double >("parallel/linesMinAngle", lines_min_angle_, 0.3);
    nh_local_.param< double >("parallel/linesMaxAngle", lines_max_angle_, 0.3);
    nh_local_.param< double >("parallel/charging_port", charging_port_, 1.1);
    nh_local_.param< double >("parallel/charging_port_delta_y", charging_port_delta_y_, 0.0);
    nh_local_.param< double >("parallel/charging_port_delta_angle", charging_port_delta_angle_, 0.0);
    nh_local_.param< double >("parallel/delta_backDist", delta_backDist_, 0.0);

    return false;
  }
  return true;
}
