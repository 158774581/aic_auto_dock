#include "aic_auto_dock/elevator_extra.h"

ElevatorExtra::ElevatorExtra(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : AicExtraction(nh, nh_local)
{
  extra_range_marker_ = nh_local_.advertise< visualization_msgs::Marker >("elevator/extra_range", 10);
  elevator_rectangle_marker_ = nh_local_.advertise< visualization_msgs::Marker >("elevator/rectangle", 10);

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

void ElevatorExtra::loadParam()
{
  nh_local_.param< string >("elevator/scan_topic1", scan_topic1_, "scan");
  nh_local_.param< string >("elevator/scan_frame1", scan_frame1_, "laser_link");
  nh_local_.param< string >("elevator/scan_topic2", scan_topic2_, "scan");
  nh_local_.param< string >("elevator/scan_frame2", scan_frame2_, "laser_link");
  nh_local_.param< string >("elevator/scan_topic", scan_topic_single_, "scan");
  nh_local_.param< string >("elevator/scan_frame", scan_frame_single_, "laser_link");
  nh_local_.param< double >("elevator/linesMinGap", lines_min_gap_, 0.3);
  nh_local_.param< double >("elevator/linesMaxGap", lines_max_gap_, 0.3);
  nh_local_.param< double >("elevator/maxmunLength", maxmun_length_, 0.3);
  nh_local_.param< double >("elevator/minimunLength", minimun_length_, 0.3);
  nh_local_.param< double >("elevator/linesMinAngle", lines_min_angle_, 0.3);
  nh_local_.param< double >("elevator/linesMaxAngle", lines_max_angle_, 0.3);
  nh_local_.param< double >("elevator/maxSplitLines", max_split_lines_, 0.3);
  nh_local_.param< double >("elevator/corner_min_angle", corner_min_angle_, 0.3);
  nh_local_.param< double >("elevator/corner_max_angle", corner_max_angle_, 0.3);
  nh_local_.param< double >("elevator/room_max_gap", room_max_gap_, 0.3);
  nh_local_.param< double >("elevator/room_min_gap", room_min_gap_, 0.3);
  nh_local_.param< double >("elevator/room_max_line_length", room_max_line_length_, 0.3);
  nh_local_.param< double >("elevator/room_min_line_length", room_min_line_length_, 0.3);
  nh_local_.param< double >("elevator/room_delta_y", room_delta_y_, 0.3);
  nh_local_.param< double >("elevator/elevator_depth", elevator_depth_, 0.3);
  nh_local_.param< double >("elevator/wall_min_length", wall_max_length_, 0.3);
  nh_local_.param< double >("elevator/wall_max_length", wall_max_length_, 0.3);
}

void ElevatorExtra::initParam()
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

bool ElevatorExtra::run(const geometry_msgs::Pose& old_middle_point, geometry_msgs::Pose& new_middle_point)
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
      ROS_INFO_THROTTLE(1, "elevator line extra no lines");
      ROS_DEBUG_NAMED("docking", "elevator line extra no lines");
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

    bool extralineMark = false, extraRoomMark = false;
    if (!extraLines(input_extra_lines, line_index, vec_middle_point, temp_line_index))
    {
      ROS_INFO_THROTTLE(1, "elevator extra line failed");
      ROS_DEBUG_NAMED("docking", "elevator extra line failed");

      vec_middle_point.clear();
      temp_line_index.clear();
      if (!extraWallLines(input_extra_lines, line_index, vec_middle_point, temp_line_index))
      {
        ROS_INFO_THROTTLE(1, "elevator extra line failed");
        ROS_DEBUG_NAMED("docking", "elevator extra line failed");

        if(!far_detection_)
        {
          vec_middle_point.clear();
          temp_line_index.clear();

          if (!extraRoomLines(input_extra_lines, line_index, vec_middle_point, temp_line_index))
          {
            ROS_INFO_THROTTLE(1, "elevator extra room failed");
            ROS_DEBUG_NAMED("docking", "elevator extra room failed");
            return false;
          }
          else
            extraRoomMark = true;
        }
        else
          return false;
      }
      else
        extralineMark = true;
    }
    else
      extralineMark = true;

    /*** 转换坐标系 ***/
    if (vec_middle_point.size() == 1)
    {
      ROS_INFO_THROTTLE(1, "elevator extra one middle point");
      ROS_DEBUG_NAMED("docking", "elevator extra one middle point");

      if (extralineMark)
      {
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, charging_port_delta_angle_);
        tf::Transform temp_frame;
        temp_frame.setRotation(q);
        temp_frame.setOrigin(tf::Vector3(0.0, charging_port_delta_y_, 0.0));

        tf::Transform new_laserNav_middle_frame;
        new_middle_point = vec_middle_point.front();
        tf::poseMsgToTF(new_middle_point, new_laserNav_middle_frame);
        tf::Transform new_laserNav_middleNav_frame = new_laserNav_middle_frame * temp_frame;

        q.setRPY(0.0, 0.0, 0.0);
        temp_frame.setRotation(q);
        temp_frame.setOrigin(tf::Vector3(charging_port_ + delta_backDist_, 0.0, 0.0));

        tf::Transform new_foot_middle_frame =
            foot_laser_frame_ * laser_laserNav_frame_ * new_laserNav_middleNav_frame * temp_frame;

        tf::Transform new_odom_middle_frame = odom_foot_frame_ * new_foot_middle_frame;
        br_.sendTransform(tf::StampedTransform(new_odom_middle_frame, ros::Time::now(), "odom", "new_Middle_frame"));

        tf::poseTFToMsg(new_odom_middle_frame, new_middle_point);
      }
      else if (extraRoomMark)
      {
        ROS_DEBUG_NAMED("docking", "only new middle pt");
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, charging_port_delta_angle_);
        tf::Transform temp_frame;
        temp_frame.setRotation(q);
        temp_frame.setOrigin(
            tf::Vector3(fabs(elevator_depth_) - fabs(charging_port_ + delta_backDist_), room_delta_y_, 0.0));

        tf::Transform temp_laserNav_middle_frame;
        tf::poseMsgToTF(vec_middle_point.front(), temp_laserNav_middle_frame);
        temp_laserNav_middle_frame *= temp_frame;
        tf::Transform new_foot_middle_frame = foot_laser_frame_ * laser_laserNav_frame_ * temp_laserNav_middle_frame;
        tf::Transform new_odom_middle_frame = odom_foot_frame_ * new_foot_middle_frame;

        br_.sendTransform(
            tf::StampedTransform(new_odom_middle_frame, ros::Time::now(), "odom", "room_new_Middle_frame"));
        tf::poseTFToMsg(new_odom_middle_frame, new_middle_point);
        return true;
      }
      else
        return false;

      return true;
    }
    else if (vec_middle_point.size() == 0)
    {
      ROS_INFO_THROTTLE(1, "elevator extra no middle point");
      ROS_DEBUG_NAMED("docking", "elevator extra no middle point");

      return false;
    }
    else
    {
      ROS_INFO_THROTTLE(1, "elevator extra more than one middle point");
      ROS_DEBUG_NAMED("docking", "elevator extra more than one middle point");
      return false;
    }
  }
  return false;
}

void ElevatorExtra::initLaserNav()
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

bool ElevatorExtra::infoPreprocess(const aic_laser_feature_line_extraction::LineSegmentList& lines,
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
    ROS_INFO_THROTTLE(1, "elevator line extra: no lines in the extra range");
    ROS_DEBUG_NAMED("docking", "elevator line extra: no lines in the extra range");
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

bool ElevatorExtra::extraLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                               const vector< unsigned int >& line_index,
                               vector< geometry_msgs::Pose >& vec_middle_point,
                               vector< pair< int, int > >& temp_line_index)
{
  ROS_INFO_THROTTLE(1, "open door precise");
  ROS_DEBUG_NAMED("docking", "open door precise");

  /***
   * 计算每条直线的中点
   ***/
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

  /***
   * 寻找间隔合适的两条平行直线
   ***/
  vector< pair< int, int > > vec_gap_legal_index;
  for (int i = 0; i < input_extra_lines.size(); i++)
  {
    for (int j = i; j < input_extra_lines.size(); j++)
    {
      recognize_extraction::Point start_i = {input_extra_lines.at(i).start_x, input_extra_lines.at(i).start_y};
      recognize_extraction::Point end_i = {input_extra_lines.at(i).end_x, input_extra_lines.at(i).end_y};
      recognize_extraction::Point pt_j = {vec_pt_middle.at(j).x, vec_pt_middle.at(j).y};
      double gap1 = recognize_extraction::distPtLine(start_i, end_i, pt_j);

      recognize_extraction::Point start_j = {input_extra_lines.at(j).start_x, input_extra_lines.at(j).start_y};
      recognize_extraction::Point end_j = {input_extra_lines.at(j).end_x, input_extra_lines.at(j).end_y};
      recognize_extraction::Point pt_i = {vec_pt_middle.at(i).x, vec_pt_middle.at(i).y};
      double gap2 = recognize_extraction::distPtLine(start_j, end_j, pt_i);

      ROS_DEBUG_NAMED("docking", "open door precise gap1:%f, gap2:%f", gap1, gap2);

      if (gap1 > lines_min_gap_ && gap1 < lines_max_gap_ && gap2 > lines_min_gap_ && gap2 < lines_max_gap_)
      {
        ROS_DEBUG_NAMED("docking", "open door legal precise gap1:%f, gap2:%f", gap1, gap2);

        pair< int, int > door_width_index;
        door_width_index.first = line_index.at(i);
        door_width_index.second = line_index.at(j);

        vec_gap_legal_index.push_back(door_width_index);
      }
    }
  }
  if (vec_gap_legal_index.empty())
  {
    ROS_INFO_THROTTLE(1, "elelvator line extra: there are't legal gap between lines");
    ROS_DEBUG_NAMED("docking", "elevator line extra: there are't legal gap between lines");
    return false;
  }

  /***
   * 检验两条直线时候合法
   ***/
  bool mark = false;
  for (vector< pair< int, int > >::iterator cit = vec_gap_legal_index.begin(); cit != vec_gap_legal_index.end(); ++cit)
  {
    /***/
    pair< int, int > right_index, left_index;
    if ((input_extra_lines[(*cit).first].angle < 0) && (input_extra_lines[(*cit).second].angle > 0))
    {
      if (cit->first == line_index.front())
      {
        ROS_DEBUG_NAMED("docking", "first line right: No elements in front");
        continue;
      }
      else if (cit->second == line_index.back())
      {
        ROS_DEBUG_NAMED("docking", "second line left: No elements behind");
        continue;
      }

      ROS_DEBUG_NAMED("docking", "first line right and second line left");
      right_index.first = (*cit).first;
      right_index.second = (*cit).first - 1;
      left_index.first = (*cit).second;
      left_index.second = (*cit).second + 1;
    }
    else if ((input_extra_lines[(*cit).first].angle > 0) && (input_extra_lines[(*cit).second].angle < 0))
    {
      if (cit->first == line_index.back())
      {
        ROS_DEBUG_NAMED("docking", "first line left: No elements behind");
        continue;
      }
      else if (cit->second == line_index.front())
      {
        ROS_DEBUG_NAMED("docking", "second line right: No elements in front");
        continue;
      }

      ROS_DEBUG_NAMED("docking", "first line left and second line right");
      right_index.first = (*cit).second;
      right_index.second = (*cit).second - 1;
      left_index.first = (*cit).first;
      left_index.second = (*cit).first + 1;
    }

    recognize_extraction::Point right_passage_start = {input_extra_lines.at(right_index.first).start_x,
                                                       input_extra_lines.at(right_index.first).start_y};
    recognize_extraction::Point right_passage_end = {input_extra_lines.at(right_index.first).end_x,
                                                     input_extra_lines.at(right_index.first).end_y};
    recognize_extraction::Point right_wall_start = {input_extra_lines.at(right_index.second).start_x,
                                                    input_extra_lines.at(right_index.second).start_y};
    recognize_extraction::Point right_wall_end = {input_extra_lines.at(right_index.second).end_x,
                                                  input_extra_lines.at((*cit).first).end_y};
    recognize_extraction::Point left_passage_start = {input_extra_lines.at(left_index.first).start_x,
                                                      input_extra_lines.at(left_index.first).start_y};
    recognize_extraction::Point left_passage_end = {input_extra_lines.at(left_index.first).end_x,
                                                    input_extra_lines.at(left_index.first).end_y};
    recognize_extraction::Point left_wall_start = {input_extra_lines.at(left_index.second).start_x,
                                                   input_extra_lines.at(left_index.second).start_y};
    recognize_extraction::Point left_wall_end = {input_extra_lines.at(left_index.second).end_x,
                                                 input_extra_lines.at(left_index.second).end_y};

    //    tf::Quaternion q(0, 0, 0);
    //    tf::Transform line_i_frame(q, tf::Vector3(start_i.x, start_i.y, 0));
    //    tf::Transform line_j_frame(q, tf::Vector3(start_j.x, start_j.y, 0));
    //    br_.sendTransform(tf::StampedTransform(line_i_frame, ros::Time::now(), "base_footprint", "line_i"));
    //    br_.sendTransform(tf::StampedTransform(line_j_frame, ros::Time::now(), "base_footprint", "line_j"));

    /*** 角度判断 ***/
    double angle_right = recognize_extraction::getAngelOfVectorM_PI(right_passage_start, right_passage_end,
                                                                    right_wall_start, right_wall_end);
    double angle_left = recognize_extraction::getAngelOfVectorM_PI(left_passage_start, left_passage_end,
                                                                   left_wall_start, left_wall_end);

    bool bool_angle = fabs(angle_right) > corner_min_angle_ && fabs(angle_right) < corner_max_angle_ &&
                      fabs(angle_left) > corner_min_angle_ && fabs(angle_left) < corner_max_angle_;

    //    double k1 = (input_extra_lines[(*cit).first].end_y - input_extra_lines[(*cit).first].start_y) /
    //                (input_extra_lines[(*cit).first].end_x - input_extra_lines[(*cit).first].start_x);
    //    double k2 = (input_extra_lines[(*cit).second].end_y - input_extra_lines[(*cit).second].start_y) /
    //                (input_extra_lines[(*cit).second].end_x - input_extra_lines[(*cit).second].start_x);
    //    double angle = recognize_extraction::RadToDeg(atan(fabs((k2 - k1) / (1 + k1 * k2))));
    //    bool bool_angle;
    //    if (angle > 0)
    //      bool_angle = (angle < fabs(max_door_passage_angle_) && angle > fabs(min_door_passage_angle_));
    //    else
    //      bool_angle = (angle > -fabs(max_door_passage_angle_) && angle < -fabs(min_door_passage_angle_));

    /*** 判断两直线是否连续的 ***/
    double dist1 = recognize_extraction::distance(right_passage_start, right_wall_end);
    double dist2 = recognize_extraction::distance(right_wall_start, right_passage_end);
    bool bool_splict_right = dist1 < max_split_lines_ || dist2 < max_split_lines_;

    double dist3 = recognize_extraction::distance(left_passage_start, left_wall_end);
    double dist4 = recognize_extraction::distance(left_wall_start, left_passage_end);
    bool bool_splict_left = dist3 < max_split_lines_ || dist4 < max_split_lines_;

    /*** 判断提取的直线长度是否合法 ***/
    double len1 = recognize_extraction::distance(right_passage_start, right_passage_end);
    double len2 = recognize_extraction::distance(right_wall_start, right_wall_end);
    double len3 = recognize_extraction::distance(left_passage_start, left_passage_end);
    double len4 = recognize_extraction::distance(left_wall_start, left_wall_end);

    bool bool_len =
        (len1 > minimun_length_ && len2 > minimun_length_) && len3 > minimun_length_ && len4 > minimun_length_;

    ROS_INFO_THROTTLE(
        1, "openDoor precise bool_angle:%d, angle_right:%f, angle_left:%f, // bool_len:%d, len1:%f, "
           "len2:%f // bool_splict_right:%d, dist1:%f,  dist2:%f,  bool_splict_right:%d, dist3:%f,  dist4:%f",
        bool_angle, angle_right, angle_left, bool_len, len1, len2, bool_splict_right, dist1, dist2, bool_splict_left,
        dist3, dist4);
    ROS_DEBUG_NAMED("docking",
                    "openDoor precise bool_angle:%d, angle_right:%f, angle_left:%f, // bool_len:%d, len1:%f, "
                    "len2:%f // bool_splict_right:%d, dist1:%f,  dist2:%f,  bool_splict_right:%d, dist3:%f,  dist4:%f",
                    bool_angle, angle_right, angle_left, bool_len, len1, len2, bool_splict_right, dist1, dist2,
                    bool_splict_left, dist3, dist4);

    if (bool_angle && bool_len && bool_splict_right && bool_splict_left)
    {
      ROS_INFO_THROTTLE(
          1, "openDoor precise legal bool_angle:%d, angle_right:%f, angle_left:%f, // bool_len:%d, len1:%f, "
             "len2:%f // bool_splict_right:%d, dist1:%f,  dist2:%f,  bool_splict_right:%d, dist3:%f,  dist4:%f",
          bool_angle, angle_right, angle_left, bool_len, len1, len2, bool_splict_right, dist1, dist2, bool_splict_left,
          dist3, dist4);
      ROS_DEBUG_NAMED(
          "docking", "openDoor precise legal bool_angle:%d, angle_right:%f, angle_left:%f, // bool_len:%d, len1:%f, "
                     "len2:%f // bool_splict_right:%d, dist1:%f,  dist2:%f,  bool_splict_right:%d, dist3:%f,  dist4:%f",
          bool_angle, angle_right, angle_left, bool_len, len1, len2, bool_splict_right, dist1, dist2, bool_splict_left,
          dist3, dist4);

      vector< geometry_msgs::Pose2D > pts;
      if (!(recognize_extraction::getCross(right_passage_start.x, right_passage_start.y, right_passage_end.x,
                                           right_passage_end.y, right_wall_start.x, right_wall_start.y,
                                           right_wall_end.x, right_wall_end.y, pts) &&
            recognize_extraction::getCross(left_passage_start.x, left_passage_start.y, left_passage_end.x,
                                           left_passage_end.y, left_wall_start.x, left_wall_start.y, left_wall_end.x,
                                           left_wall_end.y, pts)))
      {
        ROS_DEBUG_NAMED("docking", "feature matching actionlib server: Error when calculating straight slope");
        continue;
      }

      geometry_msgs::Pose2D pt_right, pt_left, pt_middle;
      pt_right = pts.at(0);
      pt_left = pts.at(1);
      recognize_extraction::middle_pt(pt_right, pt_left, pt_middle);

      vector< double > angles;
      //      angles.push_back(input_extra_lines[right_index.first].angle);
      //      angles.push_back(input_extra_lines[left_index.first].angle);
      //      pt_middle.theta = angles::normalize_angle(recognize_extraction::getAverageAngle(angles));

      angles.push_back(input_extra_lines[right_index.second].angle);
      angles.push_back(input_extra_lines[left_index.second].angle);
      pt_middle.theta = angles::normalize_angle(recognize_extraction::getAverageAngle(angles));

      if (fabs(pt_middle.theta) < M_PI_2)
        pt_middle.theta = angles::normalize_angle(pt_middle.theta + M_PI);

      tf::Quaternion q1(0, 0, input_extra_lines[right_index.first].angle);
      tf::Transform line_i_frame(q1, tf::Vector3(pt_right.x, pt_right.y, 0));
      tf::Quaternion q2(0, 0, input_extra_lines[left_index.first].angle);
      tf::Transform line_j_frame(q2, tf::Vector3(pt_left.x, pt_left.y, 0));
      tf::Quaternion q3(0, 0, pt_middle.theta);
      tf::Transform line_k_frame(q3, tf::Vector3(pt_middle.x, pt_middle.y, 0));
      br_.sendTransform(
          tf::StampedTransform(laser_laserNav_frame_ * line_i_frame, ros::Time::now(), scan_frame_, "right"));
      br_.sendTransform(
          tf::StampedTransform(laser_laserNav_frame_ * line_j_frame, ros::Time::now(), scan_frame_, "left"));
      br_.sendTransform(
          tf::StampedTransform(laser_laserNav_frame_ * line_k_frame, ros::Time::now(), scan_frame_, "middle"));

      tf::Quaternion q;
      q.setRPY(0, 0, pt_middle.theta);
      tf::Transform laserNav_middle_frame(q, tf::Vector3(pt_middle.x, pt_middle.y, 0.0));

      geometry_msgs::Pose middle_point;
      tf::poseTFToMsg(laserNav_middle_frame, middle_point);
      vec_middle_point.push_back(middle_point);

      std::pair< int, int > passage_index;
      passage_index.first = right_index.first;
      passage_index.second = left_index.first;

      temp_line_index.push_back(passage_index);

      mark = true;
    }
  }
  return mark;
}

bool ElevatorExtra::extraWallLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                                   const vector< unsigned int >& line_index,
                                   vector< geometry_msgs::Pose >& vec_middle_point,
                                   vector< pair< int, int > >& temp_line_index)
{
  ROS_INFO_THROTTLE(1, "wall extra");
  ROS_DEBUG_NAMED("docking", "wall extra");

  /***
   * 计算每条直线的中点
   ***/
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

  /***
   * 检验两条直线是否合法
   ***/
  vector< pair< int, int > > vec_length_legal_index;
  for (int i = 0; i < input_extra_lines.size(); i++)
  {
    for (int j = i + 1; j < input_extra_lines.size(); j++)
    {
      double len1 = sqrt(pow((input_extra_lines[i].end_x - input_extra_lines[i].start_x), 2) +
                         pow((input_extra_lines[i].end_y - input_extra_lines[i].start_y), 2));
      double len2 = sqrt(pow((input_extra_lines[j].end_x - input_extra_lines[j].start_x), 2) +
                         pow((input_extra_lines[j].end_y - input_extra_lines[j].start_y), 2));

      bool bool_len =
          (len1 > wall_min_length_ && len1 < wall_max_length_ && len2 > wall_min_length_ && len2 < wall_max_length_);

      double k1 = (input_extra_lines[i].end_y - input_extra_lines[i].start_y) /
                  (input_extra_lines[i].end_x - input_extra_lines[i].start_x);
      double k2 = (input_extra_lines[j].end_y - input_extra_lines[j].start_y) /
                  (input_extra_lines[j].end_x - input_extra_lines[j].start_x);
      double angle = atan(fabs((k2 - k1) / (1 + k1 * k2)));

      bool bool_angle = (angle < lines_max_angle_ && angle > lines_min_angle_);

      recognize_extraction::Point start_i = {input_extra_lines.at(i).start_x, input_extra_lines.at(i).start_y};
      recognize_extraction::Point end_i = {input_extra_lines.at(i).end_x, input_extra_lines.at(i).end_y};
      recognize_extraction::Point pt_j = {vec_pt_middle.at(j).x, vec_pt_middle.at(j).y};
      double gap1 = recognize_extraction::distPtLine(start_i, end_i, pt_j);

      recognize_extraction::Point start_j = {input_extra_lines.at(j).start_x, input_extra_lines.at(j).start_y};
      recognize_extraction::Point end_j = {input_extra_lines.at(j).end_x, input_extra_lines.at(j).end_y};
      recognize_extraction::Point pt_i = {vec_pt_middle.at(i).x, vec_pt_middle.at(i).y};
      double gap2 = recognize_extraction::distPtLine(start_j, end_j, pt_i);

      bool bool_collinear = (fabs(gap1) < 0.06 && fabs(gap2) < 0.06) ? true : false;

      ROS_DEBUG_NAMED("docking", "bool_angle:%d, angle:%f, bool_len:%d, len1:%f, len2:%f", bool_angle, angle, bool_len,
                      len1, len2);

      if (bool_angle && bool_len && bool_collinear)
      {
        ROS_DEBUG_NAMED("docking", "legal bool_angle:%d, angle:%f, bool_len:%d, len1:%f, len2:%f", bool_angle, angle,
                        bool_len, len1, len2);

        pair< int, int > wall_index;
        wall_index.first = line_index.at(i);
        wall_index.second = line_index.at(j);
        vec_length_legal_index.push_back(wall_index);
      }
    }
  }
  if (vec_length_legal_index.empty())
  {
    ROS_INFO_THROTTLE(1, "parallel line extra: Lack of appropriate length and angle");
    ROS_DEBUG_NAMED("docking", "parallel line extra: Lack of appropriate length and angle");
    return false;
  }

  /***
   * 寻找间隔合适的两条平行直线
   ***/
  bool mark = false;
  for (vector< pair< int, int > >::iterator cit = vec_length_legal_index.begin(); cit != vec_length_legal_index.end();
       ++cit)
  {
    recognize_extraction::Point start_i = {input_extra_lines[(*cit).first].start_x,
                                           input_extra_lines[(*cit).first].start_y};
    recognize_extraction::Point end_i = {input_extra_lines[(*cit).first].end_x, input_extra_lines[(*cit).first].end_y};

    recognize_extraction::Point start_j = {input_extra_lines[(*cit).second].start_x,
                                           input_extra_lines[(*cit).second].start_y};
    recognize_extraction::Point end_j = {input_extra_lines[(*cit).second].end_x,
                                         input_extra_lines[(*cit).second].end_y};
    double gap1 = recognize_extraction::distance(start_i, end_j);
    double gap2 = recognize_extraction::distance(start_j, end_i);
    double gap = (gap1 < gap2) ? gap1 : gap2;

    ROS_DEBUG_NAMED("docking", "open door precise gap1:%f, gap2:%f", gap1, gap2);

    if (gap > lines_min_gap_ && gap < lines_max_gap_)
    {
      ROS_DEBUG_NAMED("docking", "open door legal precise gap1:%f, gap2:%f", gap1, gap2);

      geometry_msgs::Pose2D pt1, pt2, pt_middle;
      if (gap1 < gap2)
      {
        pt1.x = start_i.x;
        pt1.y = start_i.y;
        pt1.theta = input_extra_lines[(*cit).first].angle;

        pt2.x = end_j.x;
        pt2.y = end_j.y;
        pt2.theta = input_extra_lines[(*cit).second].angle;
      }
      else
      {
        pt1.x = start_j.x;
        pt1.y = start_j.y;
        pt1.theta = input_extra_lines[(*cit).second].angle;

        pt2.x = end_i.x;
        pt2.y = end_i.y;
        pt2.theta = input_extra_lines[(*cit).first].angle;
      }
      recognize_extraction::middle_pt(pt1, pt2, pt_middle);

      vector< double > angles;
      angles.push_back(pt1.theta);
      angles.push_back(pt2.theta);
      pt_middle.theta = angles::normalize_angle(recognize_extraction::getAverageAngle(angles));
      if (fabs(pt_middle.theta) < M_PI_2)
        pt_middle.theta = angles::normalize_angle(pt_middle.theta + M_PI);

      tf::Quaternion q1(0, 0, pt1.theta);
      tf::Transform line_i_frame(q1, tf::Vector3(pt1.x, pt1.y, 0));
      tf::Quaternion q2(0, 0, pt2.theta);
      tf::Transform line_j_frame(q2, tf::Vector3(pt2.x, pt2.y, 0));
      tf::Quaternion q3(0, 0, pt_middle.theta);
      tf::Transform line_k_frame(q3, tf::Vector3(pt_middle.x, pt_middle.y, 0));
      //      br_.sendTransform(
      //          tf::StampedTransform(laser_laserNav_frame_ * line_i_frame, ros::Time::now(), scan_frame_, "right"));
      //      br_.sendTransform(
      //          tf::StampedTransform(laser_laserNav_frame_ * line_j_frame, ros::Time::now(), scan_frame_, "left"));
      br_.sendTransform(
          tf::StampedTransform(laser_laserNav_frame_ * line_k_frame, ros::Time::now(), scan_frame_, "middle"));

      tf::Quaternion q;
      q.setRPY(0, 0, pt_middle.theta);
      tf::Transform laserNav_middle_frame(q, tf::Vector3(pt_middle.x, pt_middle.y, 0.0));

      geometry_msgs::Pose middle_point;
      tf::poseTFToMsg(laserNav_middle_frame, middle_point);
      vec_middle_point.push_back(middle_point);

      temp_line_index.push_back(*cit);

      mark = true;
    }
  }
  return mark;
}

bool ElevatorExtra::extraRoomLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                                   const vector< unsigned int >& line_index,
                                   vector< geometry_msgs::Pose >& vec_middle_point,
                                   vector< pair< int, int > >& temp_line_index)
{
  ROS_INFO_THROTTLE(1, "room extra");
  ROS_DEBUG_NAMED("docking", "room extra");

  /***
   * 计算每条直线的中点
   ***/
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

  /***
   * 寻找间隔合适的两条平行直线
   ***/
  vector< pair< int, int > > vec_gap_legal_index;
  for (int i = 0; i < input_extra_lines.size(); i++)
  {
    for (int j = i; j < input_extra_lines.size(); j++)
    {
      recognize_extraction::Point start_i = {input_extra_lines.at(i).start_x, input_extra_lines.at(i).start_y};
      recognize_extraction::Point end_i = {input_extra_lines.at(i).end_x, input_extra_lines.at(i).end_y};
      recognize_extraction::Point pt_j = {vec_pt_middle.at(j).x, vec_pt_middle.at(j).y};
      double gap1 = recognize_extraction::distPtLine(start_i, end_i, pt_j);

      recognize_extraction::Point start_j = {input_extra_lines.at(j).start_x, input_extra_lines.at(j).start_y};
      recognize_extraction::Point end_j = {input_extra_lines.at(j).end_x, input_extra_lines.at(j).end_y};
      recognize_extraction::Point pt_i = {vec_pt_middle.at(i).x, vec_pt_middle.at(i).y};
      double gap2 = recognize_extraction::distPtLine(start_j, end_j, pt_i);

      ROS_DEBUG_NAMED("docking", "open door precise gap1:%f, gap2:%f", gap1, gap2);

      if (gap1 > room_min_gap_ && gap1 < room_max_gap_ && gap2 > room_min_gap_ && gap2 < room_max_gap_)
      {
        ROS_DEBUG_NAMED("docking", "open door precise gap1:%f, gap2:%f", gap1, gap2);

        pair< int, int > room_width_index;
        room_width_index.first = line_index.at(i);
        room_width_index.second = line_index.at(j);

        vec_gap_legal_index.push_back(room_width_index);
      }
    }
  }
  if (vec_gap_legal_index.empty())
  {
    ROS_INFO_THROTTLE(1, "elelvator room extra: there are't legal gap between lines");
    ROS_DEBUG_NAMED("docking", "elevator room extra: there are't legal gap between lines");
    return false;
  }

  /***
   * 检验两条直线时候合法
   ***/
  vector< pair< int, int > > vec_length_legal_index;
  for (vector< pair< int, int > >::iterator cit = vec_gap_legal_index.begin(); cit != vec_gap_legal_index.end(); ++cit)
  {
    double len1 = sqrt(pow((input_extra_lines[(*cit).first].end_x - input_extra_lines[(*cit).first].start_x), 2) +
                       pow((input_extra_lines[(*cit).first].end_y - input_extra_lines[(*cit).first].start_y), 2));
    double len2 = sqrt(pow((input_extra_lines[(*cit).second].end_x - input_extra_lines[(*cit).second].start_x), 2) +
                       pow((input_extra_lines[(*cit).second].end_y - input_extra_lines[(*cit).second].start_y), 2));

    bool bool_len = (len1 > room_min_line_length_ && len1 < room_max_line_length_ && len2 > room_min_line_length_ &&
                     len2 < room_max_line_length_);

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

    tf::Transform pt1_middle_frame;
    pt1_middle_frame.setOrigin(tf::Vector3(pt1_pt2_frame.getOrigin().getX() / 2, 0.0, 0.0));
    pt1_middle_frame.setRotation(q);

    //    double angle1 = (input_extra_lines.at(cit->first).angle + input_extra_lines.at(cit->second).angle) / 2 + M_PI;

    vector< double > angles;
    angles.push_back(input_extra_lines.at(cit->first).angle);
    angles.push_back(input_extra_lines.at(cit->second).angle);
    double angle1 = angles::normalize_angle(recognize_extraction::getAverageAngle(angles));
    if (fabs(angle1) < M_PI_2)
      angle1 = angles::normalize_angle(angle1 + M_PI);

    tf::Transform laserNav_middle_frame;
    laserNav_middle_frame = laserNav_pt1_frame * pt1_middle_frame;
    q.setRPY(0, 0, angle1);
    laserNav_middle_frame.setRotation(q);

    tf::Transform temp1 = laserNav_middle_frame.inverse() * laserNav_pt1_frame;
    tf::Transform temp2 = laserNav_middle_frame.inverse() * laserNav_pt2_frame;
    double p;
    if (temp1.getOrigin().getX() > temp2.getOrigin().getX())
      p = temp2.getOrigin().getX();

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

void ElevatorExtra::point32TFToMsg(const tf::Point& tf_a, geometry_msgs::Point32& msg)
{
  msg.x = tf_a.x();
  msg.y = tf_a.y();
  msg.z = tf_a.z();
}

void ElevatorExtra::pubExtraRangeMarker()
{
  visualization_msgs::Marker marker_msg;
  marker_msg.ns = "elevator extra range";
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

void ElevatorExtra::pubElevatorRectangle(const geometry_msgs::Polygon& points, int id)
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
  elevator_rectangle_marker_.publish(marker_msg);
}

bool ElevatorExtra::setInterface(const AicExtractionInterface& interface)
{
  interface_ = interface;
  setParam(interface_.tag, interface_.name);
}

bool ElevatorExtra::setParam(std::string tag, std::string name)
{
  ROS_INFO("elevator line extra tag no:%s, name:%s", tag.c_str(), name.c_str());

  vector< std::string > no_tag;
  vector< double > lines_min_angle, lines_max_angle, lines_max_gap, lines_min_gap, maxmun_length, minimun_length,
      max_split_lines, charging_port, charging_port_delta_y, charging_port_delta_angle, delta_backDist, room_max_gap,
      room_min_gap, room_max_line_length, room_min_line_length, room_delta_y, corner_min_angle, corner_max_angle,
      elevator_depth, wall_min_length, wall_max_length;
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
              behavior_list[i].hasMember("minimunLength") && behavior_list[i].hasMember("max_split_lines") &&
              behavior_list[i].hasMember("charging_port") && behavior_list[i].hasMember("charging_port_delta_y") &&
              behavior_list[i].hasMember("charging_port_delta_angle") && behavior_list[i].hasMember("delta_backDist") &&
              behavior_list[i].hasMember("room_max_gap") && behavior_list[i].hasMember("room_min_gap") &&
              behavior_list[i].hasMember("room_max_line_length") &&
              behavior_list[i].hasMember("room_min_line_length") && behavior_list[i].hasMember("room_delta_y") &&
              behavior_list[i].hasMember("corner_min_angle") && behavior_list[i].hasMember("corner_max_angle") &&
              behavior_list[i].hasMember("elevator_depth") && behavior_list[i].hasMember("wall_min_length") &&
              behavior_list[i].hasMember("wall_max_length"))
          {
            defaultParam = false;
            no_tag.push_back(behavior_list[i]["no_tag"]);
            lines_max_angle.push_back(behavior_list[i]["max_line_angle"]);
            lines_min_angle.push_back(behavior_list[i]["min_line_angle"]);
            lines_max_gap.push_back(behavior_list[i]["max_boards_gap"]);
            lines_min_gap.push_back(behavior_list[i]["min_boards_gap"]);
            maxmun_length.push_back(behavior_list[i]["maxmunLength"]);
            minimun_length.push_back(behavior_list[i]["minimunLength"]);
            max_split_lines.push_back(behavior_list[i]["max_split_lines"]);
            charging_port.push_back(behavior_list[i]["charging_port"]);
            charging_port_delta_y.push_back(behavior_list[i]["charging_port_delta_y"]);
            charging_port_delta_angle.push_back(behavior_list[i]["charging_port_delta_angle"]);
            delta_backDist.push_back(behavior_list[i]["delta_backDist"]);
            room_max_gap.push_back(behavior_list[i]["room_max_gap"]);
            room_min_gap.push_back(behavior_list[i]["room_min_gap"]);
            room_max_line_length.push_back(behavior_list[i]["room_max_line_length"]);
            room_min_line_length.push_back(behavior_list[i]["room_min_line_length"]);
            room_delta_y.push_back(behavior_list[i]["room_delta_y"]);
            corner_min_angle.push_back(behavior_list[i]["corner_min_angle"]);
            corner_max_angle.push_back(behavior_list[i]["corner_max_angle"]);
            elevator_depth.push_back(behavior_list[i]["elevator_depth"]);
            wall_min_length.push_back(behavior_list[i]["wall_min_length"]);
            wall_max_length.push_back(behavior_list[i]["wall_max_length"]);
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
        max_split_lines_ = max_split_lines.at(i);
        charging_port_ = charging_port.at(i);
        charging_port_delta_y_ = charging_port_delta_y.at(i);
        charging_port_delta_angle_ = charging_port_delta_angle.at(i);
        delta_backDist_ = delta_backDist.at(i);
        room_max_gap_ = room_max_gap.at(i);
        room_min_gap_ = room_min_gap.at(i);
        room_max_line_length_ = room_max_line_length.at(i);
        room_min_line_length_ = room_min_line_length.at(i);
        room_delta_y_ = room_delta_y.at(i);
        corner_min_angle_ = corner_min_angle.at(i);
        corner_max_angle_ = corner_max_angle.at(i);
        elevator_depth_ = elevator_depth.at(i);
        wall_min_length_ = wall_min_length.at(i);
        wall_max_length_ = wall_max_length.at(i);
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

    nh_local_.param< double >("elevator/linesMinGap", lines_min_gap_, 0.3);
    nh_local_.param< double >("elevator/linesMaxGap", lines_max_gap_, 0.3);
    nh_local_.param< double >("elevator/maxmunLength", maxmun_length_, 0.3);
    nh_local_.param< double >("elevator/minimunLength", minimun_length_, 0.3);
    nh_local_.param< double >("elevator/linesMinAngle", lines_min_angle_, 0.3);
    nh_local_.param< double >("elevator/linesMaxAngle", lines_max_angle_, 0.3);
    nh_local_.param< double >("elevator/maxSplitLines", max_split_lines_, 0.3);
    nh_local_.param< double >("elevator/charging_port", charging_port_, 1.1);
    nh_local_.param< double >("elevator/charging_port_delta_y", charging_port_delta_y_, 0.0);
    nh_local_.param< double >("elevator/charging_port_delta_angle", charging_port_delta_angle_, 0.0);
    nh_local_.param< double >("elevator/delta_backDist", delta_backDist_, 0.0);
    nh_local_.param< double >("elevator/corner_min_angle", corner_min_angle_, 0.3);
    nh_local_.param< double >("elevator/corner_max_angle", corner_max_angle_, 0.3);
    nh_local_.param< double >("elevator/room_max_gap", room_max_gap_, 0.3);
    nh_local_.param< double >("elevator/room_min_gap", room_min_gap_, 0.3);
    nh_local_.param< double >("elevator/room_max_line_length", room_max_line_length_, 0.3);
    nh_local_.param< double >("elevator/room_min_line_length", room_min_line_length_, 0.3);
    nh_local_.param< double >("elevator/room_delta_y", room_delta_y_, 0.3);
    nh_local_.param< double >("elevator/elevator_depth", elevator_depth_, 0.3);
    nh_local_.param< double >("elevator/wall_min_length", wall_max_length_, 0.3);
    nh_local_.param< double >("elevator/wall_max_length", wall_max_length_, 0.3);

    return false;
  }
  return true;
}
