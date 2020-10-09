/*
  author :7
  date   :2020.9.18
*/
#include <aic_auto_dock/gui_way_teb.h>
#include "interpolation.h"

using namespace alglib;
//gui way 

gui_way::gui_way(ros::NodeHandle& nh, ros::NodeHandle& local_nh)
    : nh_(nh), local_nh_(local_nh), as_(new Server(local_nh, "gui_way", boost::bind(&gui_way::start, this, _1), false)),
    planner_(new teb_planner(local_nh)),simple_goal_client_(new Client_gui_way("aic_auto_dock_guiway/gui_way", true))
{
  as_->start();

  setFootprint_sub_ = nh_.subscribe("/move_base/local_costmap/set_footprint", 10, &gui_way::setFootprintCallback, this);
  laser_sub_ = nh_.subscribe("/scan", 10, &gui_way::LaserCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &gui_way::odomCallback, this);
  twist_pub_ = nh_.advertise< geometry_msgs::Twist >("/dock_cmd_vel", 10);
  //  sub_robot_exception_ = nh_.subscribe("/robot_exception", 10, &gui_way::robotExceptionCallback, this);
  marker_pub_CarRadius = local_nh_.advertise< visualization_msgs::Marker >("extra_range", 10);
  marker_pub_ = local_nh_.advertise< visualization_msgs::Marker >("extra_range", 10);
  virtual_path_pub_ = local_nh_.advertise< visualization_msgs::Marker >("virtual_path", 10);

  vector< geometry_msgs::Point > points = makeFootprintFromParams(nh);
  points_ = points;

  //teb
  simple_goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &gui_way::CB_simple_goal,this);
  //ros::Timer publish_timer = nh.createTimer(ros::Duration(0.1), \
  //                           boost::bind(&teb_planner::CB_publishCycle,planner_,_1));
  
  //end 
  double half_length = 0, half_width = 0;
  for (vector< geometry_msgs::Point >::iterator cit = points.begin(); cit != points.end(); cit++)
  {
    if (abs(cit->x) > half_length)
      half_length = abs(cit->x);
    if (abs(cit->y) > half_width)
      half_width = abs(cit->y);
  }
  half_length_ = half_length;
  half_width_ = half_width;
  accept_robotInfo_ = true;

  try
  {
    listerner_.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10.0));
    listerner_.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), foot_laser_frame_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  /***
   * init launch param
   ***/
  local_nh_.param< double >("yawThrehold", yawThrehold_, 0.02);
  local_nh_.param< double >("delta_detector_spacing", delta_detector_spacing_, 0.001);
  local_nh_.param< double >("dangerRange_x", dangerRange_x_, 0.05);
  local_nh_.param< double >("dangerRange_y", dangerRange_y_, 0.02);
  local_nh_.param< double >("radiusThrehold", radiusThrehold_, 0.1);
  local_nh_.param< double >("detectorYCoordinate", detector_y_coordinate_, 0.1);
  local_nh_.param< double >("aW_acc", aW_acc_, 0.15);
  local_nh_.param< double >("aW_dcc", aW_dcc_, 0.15);
  local_nh_.param< double >("prepare_lineVel", prepare_lineVel_, 0.1);
  local_nh_.param< double >("prepare_angleVel", prepare_angleVel_, 0.3);
  local_nh_.param< double >("prepare_scale", prepare_scale_, 0.05);
  local_nh_.param< double >("angleVel_turn", angleVel_turn_, 0.3);

  loadParamFromYaml();
}

void gui_way::start(const aic_auto_dock::gui_way2GoalConstPtr& req)
{
  laser_sub_ = nh_.subscribe("/scan", 10, &gui_way::LaserCallback, this);
  ROS_INFO("gui way accept goal");

  PoseSE2 initial_pos;
  PoseSE2 goal_pos;
  double yaw;
  tf::Quaternion q;
  /*** 初始化目标点 ***/
  tf::poseMsgToTF(req->pose, odom_port_frame_);
  /*** 方向 ***/
  if (req->type == aic_auto_dock::gui_way2Goal::STRAIGHT)
  {
    direction_ = docking_direction::forward;
    dangerRange_xMax_ = fabs(dangerRange_x_);
    dangerRange_xMin_ = -fabs(dangerRange_x_);
    dangerRange_yMax_ = fabs(dangerRange_y_);
    dangerRange_yMin_ = -fabs(dangerRange_y_);
  }
  else if (req->type == aic_auto_dock::gui_way2Goal::BACK)
  {
    direction_ = docking_direction::backward;
    dangerRange_xMax_ = fabs(dangerRange_x_);
    dangerRange_xMin_ = -fabs(dangerRange_x_);
    dangerRange_yMax_ = fabs(dangerRange_y_);
    dangerRange_yMin_ = -fabs(dangerRange_y_);
  }
  else
  {
    ROS_ERROR("gui_way: Error type! Please start first");
    aic_auto_dock::gui_way2Result result;
    result.result = aic_auto_dock::gui_way2Result::INITFAILED;
    result.remaining_distance = 0.0;
    as_->setSucceeded(result);
    laser_sub_.shutdown();
    return;
  }
  ROS_WARN("direction:%d", direction_);

  /*** 速度 ***/
  vel_line_ = req->vel_line;
  vel_angle_ = req->vel_angle;
  if ((vel_line_ == 0) || (vel_angle_ == 0))
  {
    vel_line_ = 0.1;
    vel_angle_ = 0.3;
  }

  /*** 避障区域 ***/
  points_polygon_.points.clear();
  points_straight_polygon_padding_.points.clear();
  points_turn_polygon_padding_.points.clear();
  for (vector< geometry_msgs::Point >::iterator cit = points_.begin(); cit != points_.end(); cit++)
  {
    geometry_msgs::Point32 pt_straight;
    pt_straight.x = cit->x + (cit->x >= 0 ? dangerRange_xMax_ : dangerRange_xMin_);
    pt_straight.y = cit->y + (cit->y >= 0 ? dangerRange_yMax_ : dangerRange_yMin_);

    geometry_msgs::Point32 pt_turn;
    pt_turn.x = cit->x + (cit->x >= 0 ? fabs(dangerRange_x_) : -fabs(dangerRange_x_));
    pt_turn.y = cit->y + (cit->y >= 0 ? dangerRange_yMax_ : dangerRange_yMin_);

    geometry_msgs::Point32 pt1;
    pt1.x = cit->x;
    pt1.y = cit->y;
    points_polygon_.points.push_back(pt1);
    points_straight_polygon_padding_.points.push_back(pt_straight);
    points_turn_polygon_padding_.points.push_back(pt_turn);
  }

  /*** 其他参数 ***/
  back_dist_ = req->back_dist;
  obstacle_dist_ = req->obstacle_dist;
  ROS_INFO("obstacle_dist_ %f back_dist_ %f", obstacle_dist_,back_dist_);
  preparePosition_ = req->preparePosition;
  if (req->scale > 0 || req->scale < 0.1)
    scale_ = req->scale;
  else
    scale_ = 0.1;

  /*** 全局环境变量 ***/
  actionlib_status_ = executing;
  motion_status_ = empty;

  double angleVel_turn = angleVel_turn_;
  geometry_msgs::Twist twist;
  ros::Rate rate(20);
  double begin_time = ros::Time::now().toSec();

  /*** 执行 ***/
  while (ros::ok())
  {
    rate.sleep();

    if (accept_laserScan)
      accept_laserScan = false;
    else
      continue;

    if (!as_->isActive())
    {
      laser_sub_.shutdown();
      return;
    }

    planner_->CB_publishCycle();
    double dt = ros::Time::now().toSec() - begin_time;
    begin_time = ros::Time::now().toSec();

    //teb planner
    tf::StampedTransform odom_foot_frame;
     /***/
    try
    {
      listerner_.waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(10.0));
      listerner_.lookupTransform("odom", "base_footprint", ros::Time(0), odom_foot_frame);
      initial_pos.x()  = odom_foot_frame.getOrigin().getX();
      initial_pos.y()  = odom_foot_frame.getOrigin().getY();
      initial_pos.theta()  = tf::getYaw(odom_foot_frame.getRotation());
    }
    catch (tf::TransformException ex)
    {
      initial_pos.x()  = 0;
      initial_pos.y()  = 0;
      initial_pos.theta()  = 0;
      ROS_ERROR("can't find transform 'odom','base_link'");
    }

    tf::Transform odom_goal_frame(odom_port_frame_);
    q.setRPY(0,0,0);
    tf::Transform goal_robot_frame;
    tf::Transform goal_nav_frame(q);
    goal_robot_frame = odom_port_frame_.inverse()*odom_foot_frame;
    double cfg_x = 1.5;
    double parabola_b = cfg_x;
    //y=a(x-b)^2
    double parabola_a = goal_robot_frame.getOrigin().getY()/pow(goal_robot_frame.getOrigin().getX()-parabola_b,2);
    nav_msgs::Path path;
    geometry_msgs::PoseStamped via_p;
    double cfg_dx = 0.1;
    double cfg_path_length = 1.5;
    double path_length = 0;
    for (double x=goal_robot_frame.getOrigin().getX();x>0;x=x-cfg_dx)
    {
      if(x < parabola_b)
      {
        goal_nav_frame.setOrigin(tf::Vector3(x,0.0,0.0));
        path_length += cfg_dx;
      }
      else
      {
        goal_nav_frame.setOrigin(tf::Vector3(x,parabola_a*pow(x-parabola_b,2),0.0));
        path_length += hypot(cfg_dx,2*parabola_a*(x-parabola_b)*cfg_dx);//dy = 2*a(x-b)*dt
      }
      odom_goal_frame  = odom_port_frame_*goal_nav_frame; 
      via_p.pose.position.x = odom_goal_frame.getOrigin().getX();
      via_p.pose.position.y = odom_goal_frame.getOrigin().getY();
      path.poses.push_back(via_p);
      if(path_length > cfg_path_length)
      {
        break;
      }
    
    planner_->setViaPoints(path);
    }
    goal_pos.x() = odom_goal_frame.getOrigin().getX();
    goal_pos.y() = odom_goal_frame.getOrigin().getY();
    goal_pos.theta() = tf::getYaw(odom_goal_frame.getRotation());
    if (req->type == aic_auto_dock::gui_way2Goal::STRAIGHT)
    {
      goal_pos.theta() = goal_pos.theta()>0? goal_pos.theta()-M_PI:goal_pos.theta()+M_PI;
    }
 
    double vy;
    planner_->setplan(initial_pos,realTime_twist_,goal_pos);

    if(planner_->getVelocityCommand(twist.linear.x,vy,twist.angular.z,1) == false)
    {
      //plan failed ,set vel = 0
      twist.linear.x  = 0;
      twist.angular.z = 0;
    }
 
    //end(teb planner)
    avoidType_ = AvoidType::RECTANGLE;

    if (hypot(initial_pos.x() - odom_port_frame_.getOrigin().getX(),\
        initial_pos.y() - odom_port_frame_.getOrigin().getY()) <= back_dist_)
    {
      motion_status_ = success;
      ROS_INFO("goal is reached !");
      break;
    }

    /*** danger detected and paused ***/
    bool avoiding = false;
    static bool avoiding_feedback = false;
    static bool last_avoiding = avoiding;
    static bool status_code_pub = false, timer = false;
    static double beginTime;
    if (actionlib_status_ == paused)
    {
      twist.angular.z = 0;
      twist.linear.x = 0.0;
    }

    if ((danger_mark_) && (target_foot_frame.getOrigin().x() > obstacle_dist_) && !(actionlib_status_ == paused))
    {
      avoiding = true;

      twist.linear.x = 0;
      twist.angular.z = 0;

      ROS_ERROR_THROTTLE(3, "find obstacle!");
    }
 
    //语音播报，时间控制
    if (avoiding && !avoiding_feedback)
    {
      beginTime = ros::Time::now().toSec();

      status_code_pub = true;
      avoiding_feedback = true;
      timer = true;
    }

    if (timer)
    {
      if (ros::Time::now().toSec() - beginTime > 2)
      {
        avoiding_feedback = false;
        timer = false;
      }
    }

    if (status_code_pub)
    {
      status_code_pub = false;

      /*** danger avoiding feedback ***/
      aic_auto_dock::gui_way2Feedback feedback;
      feedback.status = aic_auto_dock::gui_way2Feedback::EXECUTING;
      feedback.feedback = aic_auto_dock::gui_way2Feedback::OBSTACLE_AVOIDING;
      as_->publishFeedback(feedback);
    }

    if (!avoiding && last_avoiding)
    {
      aic_auto_dock::gui_way2Feedback feedback;
      feedback.status = aic_auto_dock::gui_way2Feedback::EXECUTING;
      feedback.feedback = aic_auto_dock::gui_way2Feedback::AVOID_SUCCESS;
      as_->publishFeedback(feedback);
    }
    last_avoiding = avoiding;
    /*** 剩余距离 ***/
    aic_auto_dock::gui_way2Feedback feedback;
    feedback.status = aic_auto_dock::gui_way2Feedback::EXECUTING;
    feedback.feedback = aic_auto_dock::gui_way2Feedback::STEP_PROCESS;
    feedback.remaining_distance = port_foot_frame.getOrigin().getX();
    if (process_.prepareNavStepProcessing)
      feedback.step_process = aic_auto_dock::gui_way2Feedback::PREPARE_NAV_STEP;
    else if (process_.prepareStepProcessing)
      feedback.step_process = aic_auto_dock::gui_way2Feedback::PREPARE_STEP;
    else if (process_.portStepProcessing)
      feedback.step_process = aic_auto_dock::gui_way2Feedback::PORT_STEP;
    as_->publishFeedback(feedback);

    /*** display obstacle avoidance area ***/
    if (target_foot_frame.getOrigin().x() > obstacle_dist_)
    {
      if (avoidType_ == AvoidType::ROUND)
        pubMarkerCarTurnSquare();
      else
        pubMarkerCarStraightSquare();
    }
    /***
     * cancle or new goal
     ***/
    if (!goalAccept())
    {
      twist.angular.z = 0;
      twist.linear.x = 0.0;
      twist_pub_.publish(twist);
      motion_status_ = canclegoal;
      laser_sub_.shutdown();
      return;
    }
    ROS_DEBUG_NAMED("guiway", " twist.linear.x:%f", twist.linear.x);
    twist_pub_.publish(twist);
  }

  twist.angular.z = 0;
  twist.linear.x = 0.0;
  twist_pub_.publish(twist);

  if (motion_status_ == success)
  {
    aic_auto_dock::gui_way2Result result;
    result.result = aic_auto_dock::gui_way2Result::SUCCESS;
    result.remaining_distance = 0.0;
    as_->setSucceeded(result);
    ROS_INFO("gui_way: Motion success");
    laser_sub_.shutdown();
    return;
  }
  else if ((motion_status_ == failed) || (motion_status_ == timeout))
  {
    aic_auto_dock::gui_way2Result result;
    result.result = aic_auto_dock::gui_way2Result::FAILED;
    result.remaining_distance = port_foot_frame.getOrigin().x();
    as_->setAborted(result);
    ROS_INFO("gui_way: Motion fail");
    laser_sub_.shutdown();
    return;
  }
  else if (motion_status_ == canclegoal)
  {
    laser_sub_.shutdown();
    return;
  }
  else
  {
    aic_auto_dock::gui_way2Result result;
    result.remaining_distance = port_foot_frame.getOrigin().x();
    as_->setAborted(result);
    ROS_INFO("gui_way: Motion known");
    laser_sub_.shutdown();
    return;
  }
}

void gui_way::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Vector3 vec(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  realTime_odom_.setOrigin(vec);
  realTime_odom_.setRotation(q);
  realTime_twist_ = msg->twist.twist;
}

void gui_way::setFootprintCallback(const geometry_msgs::Polygon& msg)
{
  points_.clear();
  double half_length = 0, half_width = 0;
  for (int i = 0; i < msg.points.size(); i++)
  {
    geometry_msgs::Point pt;
    pt.x = msg.points.at(i).x;
    pt.y = msg.points.at(i).y;
    points_.push_back(pt);

    if (abs(msg.points.at(i).x) > half_length)
      half_length = abs(msg.points.at(i).x);
    if (abs(msg.points.at(i).y) > half_width)
      half_width = abs(msg.points.at(i).y);
  }
  half_length_ = half_length;
  half_width_ = half_width;
  accept_robotInfo_ = true;
}

void gui_way::LaserCallback(const sensor_msgs::LaserScan& msg)
{
  if (accept_robotInfo_)
  {
    //避障模块
    for (int i = 0; i < msg.ranges.size(); i++)
    {
      double laser_num_ = msg.ranges.size();
      double dist_laser = msg.ranges[i];
      double rot_offset = (laser_num_ * msg.angle_increment) / 2;
      double angle_laser = msg.angle_increment * i - rot_offset;
      double x = dist_laser * cos(angle_laser);
      double y = dist_laser * sin(angle_laser);

      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      tf::Transform laser_pt_frame(q, tf::Vector3(x, y, 0));
      tf::Transform foot_pt_frame = foot_laser_frame_ * laser_pt_frame;

      if (avoidType_ == AvoidType::ROUND)
      {
        bool boolcarBodyPadding = pnpoly::polygon_contains(
            points_turn_polygon_padding_, foot_pt_frame.getOrigin().getX(), foot_pt_frame.getOrigin().getY());
        bool boolcarBody = pnpoly::polygon_contains(points_polygon_, foot_pt_frame.getOrigin().getX(),
                                                    foot_pt_frame.getOrigin().getY());

        if (dist_laser > msg.range_min && dist_laser < msg.range_max && !boolcarBody && boolcarBodyPadding)
        {
          danger_mark_ = true;
          accept_laserScan = true;

          return;
        }
      }
      else
      {
        bool boolcarBodyPadding = pnpoly::polygon_contains(
            points_straight_polygon_padding_, foot_pt_frame.getOrigin().getX(), foot_pt_frame.getOrigin().getY());
        bool boolcarBody = pnpoly::polygon_contains(points_polygon_, foot_pt_frame.getOrigin().getX(),
                                                    foot_pt_frame.getOrigin().getY());

        if (dist_laser > msg.range_min && dist_laser < msg.range_max && !boolcarBody && boolcarBodyPadding)
        {
          danger_mark_ = true;
          accept_laserScan = true;

          return;
        }
      }
    }

    danger_mark_ = false;
    accept_laserScan = true;
  }
  else
    ROS_INFO("move avoid node cannot get the robot information, temporarily shut down obstacle avoidance function!");
}

bool gui_way::goalAccept()
{
  if (as_->isPreemptRequested())
  {
    if (as_->isNewGoalAvailable())
    {
      aic_auto_dock::gui_way2GoalConstPtr goal = as_->acceptNewGoal();

      if (goal->type == aic_auto_dock::gui_way2Goal::PAUSE && actionlib_status_ == executing)
      {
        actionlib_status_ = paused;
        aic_auto_dock::gui_way2Feedback feedback;
        feedback.status = aic_auto_dock::gui_way2Feedback::PAUSE;
        as_->publishFeedback(feedback);
      }
      else if (goal->type == aic_auto_dock::gui_way2Goal::RESUM && actionlib_status_ == paused)
      {
        actionlib_status_ = executing;
        aic_auto_dock::gui_way2Feedback feedback;
        feedback.status = aic_auto_dock::gui_way2Feedback::EXECUTING;
        as_->publishFeedback(feedback);
      }
      else if (goal->type == aic_auto_dock::gui_way2Goal::INITPORT)
      {
        if (process_.process == StepProcess::Process::prepareStep)
        {
          if (process_.prepareNavStepProcessing || process_.prepareStepProcessing)
            return true;
        }

        tf::poseMsgToTF(goal->pose, odom_port_frame_);
      }
      else if ((goal->type == aic_auto_dock::gui_way2Goal::STRAIGHT) ||
               (goal->type == aic_auto_dock::gui_way2Goal::BACK))
      {
        ROS_ERROR("gui_way: Error! Accept new goal, but the task is still executing! Please "
                  "wait and send again");
        aic_auto_dock::gui_way2Feedback feedback;

        if (actionlib_status_ == executing)
          feedback.status = aic_auto_dock::gui_way2Feedback::EXECUTING;
        else if (actionlib_status_ == paused)
          feedback.status = aic_auto_dock::gui_way2Feedback::PAUSE;

        feedback.feedback = aic_auto_dock::gui_way2Feedback::ILLEGAL_GOAL;
        as_->publishFeedback(feedback);
      }
      else
      {
        aic_auto_dock::gui_way2Feedback feedback;

        if (actionlib_status_ == executing)
          feedback.status = aic_auto_dock::gui_way2Feedback::EXECUTING;
        else if (actionlib_status_ == paused)
          feedback.status = aic_auto_dock::gui_way2Feedback::PAUSE;

        feedback.feedback = aic_auto_dock::gui_way2Feedback::ILLEGAL_GOAL;
        as_->publishFeedback(feedback);
        ROS_ERROR("gui_way: Accept new goal error! type error!");
      }
    }
    else
    {
      geometry_msgs::Twist twist;
      twist.angular.z = 0;
      twist.linear.x = 0.0;
      twist_pub_.publish(twist);
      aic_auto_dock::gui_way2Result result;
      result.result = aic_auto_dock::gui_way2Result::CANCLE;
      result.remaining_distance = port_foot_frame.getOrigin().x();
      as_->setAborted(result);
      ROS_INFO("gui_way: Cancle goal, result.remaining_distance:%f", result.remaining_distance);
      return false;
    }
  }
  return true;
}

void gui_way::pubMarkerCarStraightSquare()
{
  visualization_msgs::Marker marker_msg;

  marker_msg.ns = "move avoid square extra_range";
  marker_msg.id = 1;
  marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
  marker_msg.scale.x = 0.03;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.4;
  marker_msg.color.b = 0.7;
  marker_msg.color.a = 1.0;

  for (vector< geometry_msgs::Point32 >::iterator cit = points_polygon_.points.begin();
       cit != points_polygon_.points.end(); cit++)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->x;
    p_start.y = cit->y;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
  }

  geometry_msgs::Point p_start;
  p_start.x = points_polygon_.points.begin()->x;
  p_start.y = points_polygon_.points.begin()->y;
  marker_msg.points.push_back(p_start);

  for (vector< geometry_msgs::Point32 >::iterator cit = points_straight_polygon_padding_.points.begin();
       cit != points_straight_polygon_padding_.points.end(); cit++)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->x;
    p_start.y = cit->y;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
  }

  p_start.x = points_straight_polygon_padding_.points.begin()->x;
  p_start.y = points_straight_polygon_padding_.points.begin()->y;
  marker_msg.points.push_back(p_start);

  marker_msg.lifetime = ros::Duration(0.1);

  marker_msg.header.frame_id = "base_footprint";
  marker_msg.header.stamp = ros::Time::now();
  marker_pub_.publish(marker_msg);
}

void gui_way::pubMarkerCarTurnSquare()
{
  visualization_msgs::Marker marker_msg;

  marker_msg.ns = "move avoid square extra_range";
  marker_msg.id = 1;
  marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
  marker_msg.scale.x = 0.03;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.4;
  marker_msg.color.b = 0.7;
  marker_msg.color.a = 1.0;

  for (vector< geometry_msgs::Point32 >::iterator cit = points_polygon_.points.begin();
       cit != points_polygon_.points.end(); cit++)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->x;
    p_start.y = cit->y;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
  }

  geometry_msgs::Point p_start;
  p_start.x = points_polygon_.points.begin()->x;
  p_start.y = points_polygon_.points.begin()->y;
  marker_msg.points.push_back(p_start);

  for (vector< geometry_msgs::Point32 >::iterator cit = points_turn_polygon_padding_.points.begin();
       cit != points_turn_polygon_padding_.points.end(); cit++)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->x;
    p_start.y = cit->y;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
  }

  p_start.x = points_turn_polygon_padding_.points.begin()->x;
  p_start.y = points_turn_polygon_padding_.points.begin()->y;
  marker_msg.points.push_back(p_start);

  marker_msg.lifetime = ros::Duration(0.1);

  marker_msg.header.frame_id = "base_footprint";
  marker_msg.header.stamp = ros::Time::now();
  marker_pub_.publish(marker_msg);
}

void gui_way::pubVirtualPath()
{
  visualization_msgs::Marker marker_msg;

  marker_msg.ns = "virtual path";
  marker_msg.id = 1;
  marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
  marker_msg.scale.x = 0.03;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.4;
  marker_msg.color.b = 0.7;
  marker_msg.color.a = 1.0;

  geometry_msgs::Point temp_p;
  temp_p.x = odom_port_frame_.getOrigin().getX();
  temp_p.y = odom_port_frame_.getOrigin().getY();
  marker_msg.points.push_back(temp_p);

  tf::Quaternion q(0, 0, 0);
  tf::Transform t(q, tf::Vector3(4, 0, 0));
  tf::Transform T = odom_port_frame_ * t;

  geometry_msgs::Point temp_p2;
  temp_p2.x = T.getOrigin().getX();
  temp_p2.y = T.getOrigin().getY();
  marker_msg.points.push_back(temp_p2);

  marker_msg.lifetime = ros::Duration(0.1);

  marker_msg.header.frame_id = "odom";
  marker_msg.header.stamp = ros::Time::now();
  virtual_path_pub_.publish(marker_msg);
}

bool gui_way::loadParamFromYaml()
{
  std::string tag = "0";
  string name = "guiway";
  vector< std::string > no_tag;
  vector< double > delta_detector_spacing, dangerRange_x, dangerRange_y, yawThrehold, radiusThrehold,
      detector_y_coordinate, aW_acc, aW_dcc, prepare_lineVel, prepare_angleVel, prepare_scale, angleVel_turn;
  XmlRpc::XmlRpcValue behavior_list;
  bool defaultParam = true;
  if (nh_.getParam(name, behavior_list))
  {
    if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < behavior_list.size(); ++i)
      {
        if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          if (behavior_list[i].hasMember("no_tag") && behavior_list[i].hasMember("delta_detector_spacing") &&
              behavior_list[i].hasMember("dangerRange_x") && behavior_list[i].hasMember("dangerRange_y") &&
              behavior_list[i].hasMember("yawThrehold") && behavior_list[i].hasMember("radiusThrehold") &&
              behavior_list[i].hasMember("detectorYCoordinate") && behavior_list[i].hasMember("aW_dcc") &&
              behavior_list[i].hasMember("aW_acc") && behavior_list[i].hasMember("prepare_lineVel") &&
              behavior_list[i].hasMember("prepare_angleVel") && behavior_list[i].hasMember("prepare_scale") &&
              behavior_list[i].hasMember("angleVel_turn"))
          {
            defaultParam = false;
            no_tag.push_back(behavior_list[i]["no_tag"]);
            delta_detector_spacing.push_back(behavior_list[i]["delta_detector_spacing"]);
            dangerRange_x.push_back(behavior_list[i]["dangerRange_x"]);
            dangerRange_y.push_back(behavior_list[i]["dangerRange_y"]);
            yawThrehold.push_back(behavior_list[i]["yawThrehold"]);
            radiusThrehold.push_back(behavior_list[i]["radiusThrehold"]);
            detector_y_coordinate.push_back(behavior_list[i]["detectorYCoordinate"]);
            aW_acc.push_back(behavior_list[i]["aW_acc"]);
            aW_dcc.push_back(behavior_list[i]["aW_dcc"]);
            prepare_lineVel.push_back(behavior_list[i]["prepare_lineVel"]);
            prepare_angleVel.push_back(behavior_list[i]["prepare_angleVel"]);
            prepare_scale.push_back(behavior_list[i]["prepare_scale"]);
            angleVel_turn.push_back(behavior_list[i]["angleVel_turn"]);
          }
          else
          {
            ROS_ERROR("guiway yaml loader must have a name and a type and this does not. Using the default yaml loader "
                      "behaviors instead.");
            defaultParam = true;
            break;
          }
        }
        else
        {
          ROS_ERROR("guiway yaml loader must be specified as maps, but they are XmlRpcType %d. We'll use the default "
                    "yaml loader instead.",
                    behavior_list[i].getType());
          defaultParam = true;
          break;
        }
      }
    }
    else
    {
      ROS_ERROR("guiway The yaml loader specification must be a list, but is of XmlRpcType %d. We'll use the default "
                "yaml loader instead.",
                behavior_list.getType());
      defaultParam = true;
    }
  }
  else
  {
    ROS_ERROR("guiway The yaml loader specification must have a specific name. We'll use the default "
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
        ROS_INFO("guiway Load yaml param: %s", tag.c_str());

        accept_targe = true;
        delta_detector_spacing_ = delta_detector_spacing.at(i);
        dangerRange_x_ = dangerRange_x.at(i);
        dangerRange_y_ = dangerRange_y.at(i);
        yawThrehold_ = yawThrehold.at(i);
        radiusThrehold_ = radiusThrehold.at(i);
        detector_y_coordinate_ = detector_y_coordinate.at(i);
        aW_acc_ = aW_acc.at(i);
        aW_dcc_ = aW_dcc.at(i);
        prepare_lineVel_ = prepare_lineVel.at(i);
        prepare_angleVel_ = prepare_angleVel.at(i);
        prepare_scale_ = prepare_scale.at(i);
        angleVel_turn_ = angleVel_turn.at(i);
        break;
      }
    }
    if (accept_targe == false)
    {
      ROS_ERROR("guiway The accepted tag no is not include in the yaml loader specification. We'll use the default "
                "yaml loader instead.");
      defaultParam = true;
    }
  }

  if (defaultParam == true)
  {
    ROS_INFO("guiway Default launch param");
    local_nh_.param< double >("yawThrehold", yawThrehold_, 0.02);
    local_nh_.param< double >("delta_detector_spacing", delta_detector_spacing_, 0.001);
    local_nh_.param< double >("dangerRange_x", dangerRange_x_, 0.05);
    local_nh_.param< double >("dangerRange_y", dangerRange_y_, 0.02);
    local_nh_.param< double >("radiusThrehold", radiusThrehold_, 0.1);
    local_nh_.param< double >("detectorYCoordinate", detector_y_coordinate_, 0.1);
    local_nh_.param< double >("aW_acc", aW_acc_, 0.15);
    local_nh_.param< double >("aW_dcc", aW_dcc_, 0.15);
    local_nh_.param< double >("prepare_lineVel", prepare_lineVel_, 0.1);
    local_nh_.param< double >("prepare_angleVel", prepare_angleVel_, 0.3);
    local_nh_.param< double >("prepare_scale", prepare_scale_, 0.05);
    local_nh_.param< double >("angleVel_turn", angleVel_turn_, 0.3);

    return false;
  }
  return true;
}
//teb 
void gui_way::CB_simple_goal(const geometry_msgs::PoseStampedConstPtr& msg)
{
  PoseSE2 goal_pos;
  tf::StampedTransform odom_map_frame;
  tf::Transform odom_goal_frame;
  tf::Transform map_goal_frame;
  tf::Quaternion q;
  q.setRPY(0,0,tf::getYaw(msg.get()->pose.orientation));
  listerner_.waitForTransform("odom", "map", ros::Time(0), ros::Duration(10.0));
  listerner_.lookupTransform("odom", "map", ros::Time(0), odom_map_frame);
  map_goal_frame.getOrigin().setValue(msg->pose.position.x,msg->pose.position.y,0);
  map_goal_frame.setRotation(q);
  odom_goal_frame  = odom_map_frame*map_goal_frame;
  goal_pos.x()                       = odom_goal_frame.getOrigin().getX();
  goal_pos.y()                       = odom_goal_frame.getOrigin().getY();
  goal_pos.theta()                   = tf::getYaw(odom_goal_frame.getRotation());
  ROS_INFO("goal pose %f %f %f",goal_pos.x(),goal_pos.y(),goal_pos.theta());

  aic_auto_dock::gui_way2Goal guiway_goal;
  geometry_msgs::Pose goal_msg;
  tf::poseTFToMsg(odom_goal_frame,goal_msg);
    // goal_msg.position.x = odom_goal_frame.getOrigin().getX();
    // goal_msg.position.y = odom_goal_frame.getOrigin().getY();
    // goal_msg.orientation.x = odom_goal_frame.getRotation().getX();
    // goal_msg.orientation.y = odom_goal_frame.getRotation().getY();
    // goal_msg.orientation.z = odom_goal_frame.getRotation().getZ();
    // goal_msg.orientation.w = odom_goal_frame.getRotation().getW();
  //aic_auto_dock::gui_way2Goal::BACK;
  //aic_auto_dock::gui_way2Goal::STRAIGHT;
  guiway_goal.type = aic_auto_dock::gui_way2Goal::STRAIGHT;
  guiway_goal.pose = goal_msg;

  guiway_goal.vel_line = 6;
  guiway_goal.vel_angle = 1;
  guiway_goal.back_dist = 0.1;//fabs(half_length_);
  guiway_goal.obstacle_dist = 0.1;//fabs(half_length_);
  //guiway_goal.preparePosition = 0.1;
  simple_goal_client_->waitForServer();
  ROS_INFO("wait server succeed");

  double cfg_x = 1;
  double cfg_y = 0.5;
  //add obstacles
  planner_->clearObstacle();
  q.setRPY(0,0,0);
  tf::Transform goal_obs_frame(q,tf::Vector3(cfg_x,-cfg_y,0));
  tf::Transform goal_obs1_frame(q,tf::Vector3(-cfg_x,-cfg_y,0));
  tf::Transform odom_obs_frame = odom_goal_frame*goal_obs_frame;
  tf::Transform odom_obs1_frame = odom_goal_frame*goal_obs1_frame;
  planner_->setLineObstacle(odom_obs_frame.getOrigin().getX(), odom_obs_frame.getOrigin().getY(),\
                            odom_obs1_frame.getOrigin().getX(), odom_obs1_frame.getOrigin().getY());

  goal_obs_frame.setOrigin(tf::Vector3(-cfg_x,-cfg_y,0));
  goal_obs1_frame.setOrigin(tf::Vector3(-cfg_x,cfg_y,0));
  odom_obs_frame = odom_goal_frame*goal_obs_frame;
  odom_obs1_frame = odom_goal_frame*goal_obs1_frame;
  planner_->setLineObstacle(odom_obs_frame.getOrigin().getX(), odom_obs_frame.getOrigin().getY(),\
                            odom_obs1_frame.getOrigin().getX(), odom_obs1_frame.getOrigin().getY());

  goal_obs_frame.setOrigin(tf::Vector3(-cfg_x,cfg_y,0));
  goal_obs1_frame.setOrigin(tf::Vector3(cfg_x,cfg_y,0));
  odom_obs_frame = odom_goal_frame*goal_obs_frame;
  odom_obs1_frame = odom_goal_frame*goal_obs1_frame;
  planner_->setLineObstacle(odom_obs_frame.getOrigin().getX(), odom_obs_frame.getOrigin().getY(),\
                            odom_obs1_frame.getOrigin().getX(), odom_obs1_frame.getOrigin().getY());

  simple_goal_client_->sendGoal(guiway_goal);
}
//end(teb)

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guiWay");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  gui_way guiWay(nh, nh_local);

  ros::MultiThreadedSpinner spin;
  spin.spin();
}



//end (gui way)

































// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
ros::Subscriber init_pose_sub;
ros::Subscriber goal_pose_sub;;
ros::Subscriber odom_sub;
ros::Subscriber scan_sub;
ros::Publisher velocity_pub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;
tf::TransformListener* p_tf_listener;
sensor_msgs::LaserScan scan;
bool scan_updated;
// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);
void CB_goal_pose(const geometry_msgs::PoseStampedConstPtr& msg);
void CB_odom(const nav_msgs::Odometry::ConstPtr& msg);
void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);
void scan_to_obs(std::vector<ObstaclePtr> &obst_vec);
PoseSE2 initial_pos(-4,0,0);
PoseSE2 goal_pos(4,0,0);
geometry_msgs::Twist g_cmd_vel;
nav_msgs::Odometry robot_odom;
geometry_msgs::Polygon footprint;
teb_planner* my_planner;
// =============== Main function =================
int main_teb( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
 
  tf::TransformListener tf_listener;
  p_tf_listener = &tf_listener;
  teb_planner teb_planner_1(n);
  my_planner = &teb_planner_1;

  my_planner->setLineObstacle(-0.5, 0.6,-0.5, 2.6);
  my_planner->setLineObstacle(0.5, 0.6,0.5, 2.6);
  my_planner->setLineObstacle(-0.5, 0.6,0.5, 0.6);

  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.05), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), \
                             boost::bind(&teb_planner::CB_publishCycle,my_planner,_1));
  //initial pose;
  tf::StampedTransform odom_foot_frame;
  p_tf_listener->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0));
  p_tf_listener->lookupTransform("odom", "base_link", ros::Time(0), odom_foot_frame);
  initial_pos.x()  = odom_foot_frame.getOrigin().getX();
  initial_pos.y()  = odom_foot_frame.getOrigin().getY();
  initial_pos.theta()  = tf::getYaw(odom_foot_frame.getRotation());
  goal_pos.x()  = odom_foot_frame.getOrigin().getX();
  goal_pos.y()  = odom_foot_frame.getOrigin().getY();
  goal_pos.theta()  = tf::getYaw(odom_foot_frame.getRotation());
  
  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);

  scan_sub = n.subscribe("/scan", 1, scan_callback);

  //init pose and goal pose
  goal_pose_sub = n.subscribe("/move_base_simple/goal", 1, CB_goal_pose);

  odom_sub = n.subscribe("/odom", 1, CB_odom);

  velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  ros::spin();

  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
   /***/
  try
  {
    tf::StampedTransform odom_foot_frame;
    p_tf_listener->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0));
    p_tf_listener->lookupTransform("odom", "base_link", ros::Time(0), odom_foot_frame);
    initial_pos.x()  = odom_foot_frame.getOrigin().getX();
    initial_pos.y()  = odom_foot_frame.getOrigin().getY();
    initial_pos.theta()  = tf::getYaw(odom_foot_frame.getRotation());
  }
  catch (tf::TransformException ex)
  {
    initial_pos.x()  = 0;
    initial_pos.y()  = 0;
    initial_pos.theta()  = 0;
  }
  //che if the goal is reached
  double dx = initial_pos.x() - goal_pos.x();
  double dy = initial_pos.y() - goal_pos.y();
  double delta_orient = initial_pos.theta() - goal_pos.theta();
  double xy_goal_tolerance = 0.05;
  double yaw_goal_tolerance = 0.01;
  if(fabs(std::sqrt(dx*dx+dy*dy)) < xy_goal_tolerance
    && fabs(delta_orient) < yaw_goal_tolerance)
  {
    my_planner->getplanner()->clearPlanner();
    ROS_INFO("goal is reached !");
    return;
  }
  //teb local planner
  //publish cmd_vel
  geometry_msgs::Twist cmd_vel;
  double vy;
  my_planner->setplan(initial_pos,robot_odom.twist.twist,goal_pos);
  if(my_planner->getVelocityCommand(cmd_vel.linear.x,vy,cmd_vel.angular.z,1) == false)
  {
    //plan failed ,set vel = 0
    cmd_vel.linear.x  = 0;
    cmd_vel.angular.z = 0;
  }
  velocity_pub.publish(cmd_vel);
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e)
{
  planner->visualize();//publish poses(path+time) and path 
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  
  if (index>=no_fixed_obstacles) 
    return;
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);
  
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
    {
      if (obst_msg->obstacles.at(i).radius == 0) 
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
                                                            obst_msg->obstacles.at(i).radius )));
      }
    }
    else
    {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                  obst_msg->obstacles.at(i).polygon.points[j].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if(!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
}

void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

void CB_goal_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  tf::StampedTransform odom_map_frame;
  tf::Transform odom_goal_frame;
  tf::Transform map_goal_frame;
  tf::Quaternion q;
  q.setRPY(0,0,tf::getYaw(msg.get()->pose.orientation));
  p_tf_listener->waitForTransform("odom", "map", ros::Time(0), ros::Duration(10.0));
  p_tf_listener->lookupTransform("odom", "map", ros::Time(0), odom_map_frame);
  map_goal_frame.getOrigin().setValue(msg->pose.position.x,msg->pose.position.y,0);
  map_goal_frame.setRotation(q);
  odom_goal_frame  = odom_map_frame*map_goal_frame;
  goal_pos.x()                       = odom_goal_frame.getOrigin().getX();
  goal_pos.y()                       = odom_goal_frame.getOrigin().getY();
  goal_pos.theta()                   = tf::getYaw(odom_goal_frame.getRotation());
  ROS_INFO("goal pose %f %f %f",goal_pos.x(),goal_pos.y(),goal_pos.theta());

}
void CB_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_odom.twist.twist.linear.x     = msg->twist.twist.linear.x;
  robot_odom.twist.twist.linear.y     = msg->twist.twist.linear.y;
  robot_odom.twist.twist.angular.z    = msg->twist.twist.angular.z;
}

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel (twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}

void scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  scan = *msg;
  scan_updated = true;
  obst_vector.clear();
  //scan_to_obs(obst_vector);

  //insert vitual linear obstacles
  if (0)
  {
    tf::Quaternion q;
    q.setRPY(0, 0, goal_pos.theta());
    tf::Transform odom_goal(q, tf::Vector3(goal_pos.x(), goal_pos.y(), 0));
    q.setRPY(0, 0, 0);
    tf::Transform goal_tmp(q, tf::Vector3(-1, -0.5, 0));
    tf::Transform goal_tmp1(q, tf::Vector3(1, -0.5, 0));
    goal_tmp            = odom_goal*goal_tmp;
    goal_tmp1           = odom_goal*goal_tmp1;
    obst_vector.push_back( boost::make_shared<LineObstacle>(goal_tmp.getOrigin().x(), goal_tmp.getOrigin().y(),goal_tmp1.getOrigin().x(), goal_tmp1.getOrigin().y()));
    goal_tmp.setOrigin(tf::Vector3(-1, 0.5, 0));
    goal_tmp.setRotation(q);
    goal_tmp1.setOrigin(tf::Vector3(1, 0.5, 0));
    goal_tmp1.setRotation(q);
    goal_tmp            = odom_goal*goal_tmp;
    goal_tmp1           = odom_goal*goal_tmp1;
    obst_vector.push_back( boost::make_shared<LineObstacle>(goal_tmp.getOrigin().x(), goal_tmp.getOrigin().y(),goal_tmp1.getOrigin().x(), goal_tmp1.getOrigin().y()));
  }
  else
  {
    obst_vector.push_back( boost::make_shared<LineObstacle>(0, 1,0, 3));
    obst_vector.push_back( boost::make_shared<LineObstacle>(1, 1,1, 3));
    obst_vector.push_back( boost::make_shared<LineObstacle>(0, 1,1, 1));
  }
}

void scan_to_obs(std::vector<ObstaclePtr> &obst_vec)
{
  
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf::StampedTransform odom_laser_frame;
  tf::Transform laser_obs_frame(q,tf::Vector3(0,0,0));
  p_tf_listener->lookupTransform("odom", scan.header.frame_id, ros::Time(0), odom_laser_frame);
  //foot_laser_frame = listener.lookupTransform("base_footprint", "laser_link", ros::Time(0));
  float angle = scan.angle_min;
  for(auto r : scan.ranges){
    float x = r * cos(angle);
    float y = r * sin(angle);
    laser_obs_frame.setOrigin(tf::Vector3(x,y,0));
    tf::Transform odom_obs_frame = odom_laser_frame * laser_obs_frame;
    x = odom_obs_frame.getOrigin().getX();
    y = odom_obs_frame.getOrigin().getY();
    angle += scan.angle_increment;
    obst_vec.push_back( boost::make_shared<PointObstacle>(x,y) );

  }
  return;
}


//------------ gui way teb ----------------
teb_planner::teb_planner(ros::NodeHandle& nh):nh_(nh)
{
  config_.loadRosParamFromNodeHandle(nh);
  //ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
    // Setup visualization
  visual_ = TebVisualizationPtr(new TebVisualization(nh, config_));
  
  // Setup robot shape model
  robot_model_ = TebLocalPlannerROS::getRobotFootprintFromParamServer(nh);

  // Setup planner (homotopy class planning or just the local teb planner)
  if (config_.hcp.enable_homotopy_class_planning)
    planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(config_, &obst_vector_, robot_model_, visual_, &via_points_));
  else
    planner_ = PlannerInterfacePtr(new TebOptimalPlanner(config_, &obst_vector_, robot_model_, visual_, &via_points_));
  
}

bool teb_planner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses)
{
  bool succeed  = planner_->plan(startpose_, endpose_,&startvel_,false); // hardcoded start and goal for testing purposes
  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");
    return false;
  }
  if(planner_.get()->getVelocityCommand(vx, vy, omega,look_ahead_poses) ==true)
  {
    double look_ahead_time = 0.3;
    if(isTrajFeasible(vx, vy, omega,look_ahead_time) == true)
    {
      return true;
    }
  }
  return false;
}

bool teb_planner::setplan(PoseSE2& startpose,geometry_msgs::Twist& startVel,PoseSE2& endpose)
{
  startpose_ = startpose;
  endpose_   = endpose;
  startvel_   = startVel;
  return true;
}

// Visualization loop
void teb_planner::CB_publishCycle(const ros::TimerEvent& e)
{
  planner_->visualize();//publish poses(path+time) and path 
  visual_->publishObstacles(obst_vector_);
  visual_->publishViaPoints(via_points_);
}
// Visualization loop
void teb_planner::CB_publishCycle()
{
  planner_->visualize();//publish poses(path+time) and path 
  visual_->publishObstacles(obst_vector_);
  visual_->publishViaPoints(via_points_);
}
// set obstacle
void teb_planner::setLineObstacle(float x0,float y0,float x1,float y1)
{
  obst_vector_.push_back( boost::make_shared<LineObstacle>(x0, y0,x1, y1));
}

// set obstacle
void teb_planner::clearObstacle()
{
  obst_vector_.clear();
}

bool teb_planner::isTrajFeasible(double vx, double vy, double omega, double look_ahead_time)
{
  PoseSE2 predict_pose(startpose_);
  // double DT = 0.1;
  // double t = 0;
  // while(t<look_ahead_time)
  // {
  //   predict_pose.theta()  += omega*look_ahead_time;
  //   predict_pose.x()      += vx*std::cos(predict_pose.theta())*DT;
  //   predict_pose.y()      += vx*std::sin(predict_pose.theta())*DT;
  //   t += DT;
  // }
  std::vector<TrajectoryPointMsg> traj;
  if(config_.hcp.enable_homotopy_class_planning == 0)
  {
    boost::shared_ptr<TebOptimalPlanner> p_planner = boost::dynamic_pointer_cast<TebOptimalPlanner>(planner_);
    p_planner.get()->getFullTrajectory(traj);
    for(auto pose:traj)
    {
      predict_pose.x()    = pose.pose.position.x;
      predict_pose.y()    = pose.pose.position.y;
      predict_pose.theta()    = tf::getYaw(pose.pose.orientation);
      for (ObstContainer::const_iterator obst = obst_vector_.begin(); obst != obst_vector_.end(); ++obst)
      {
        boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);   
        if (!pobst)
          continue;
          
        double obs_dis = robot_model_->calculateDistance(predict_pose,pobst.get());
        if(obs_dis < config_.obstacles.min_obstacle_dist)
        {
          ROS_INFO("robot is too close to obstacles,distance %f %f!!!",config_.obstacles.min_obstacle_dist,obs_dis);
          return false;
        }
      }
    }
  }
  else
  {

  }

  // for (ObstContainer::const_iterator obst = obst_vector_.begin(); obst != obst_vector_.end(); ++obst)
  // {
  //   boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);   
  //   if (!pobst)
	// 		continue;
      
  //   double obs_dis = robot_model_->calculateDistance(predict_pose,pobst.get());
  //   if(obs_dis < config_.obstacles.min_obstacle_dist)
  //   {
  //     ROS_INFO("robot is too close to obstacles,distance %f %f!!!",config_.obstacles.min_obstacle_dist,obs_dis);
  //     return false;
  //   }
  // }
  return true;
}

bool teb_planner::isTrajFeasible(PoseSE2 robot_pos, PoseSE2 goal_pos)
{
  // std::vector<TrajectoryPointMsg> traj;  
  // PoseSE2 predict_pose(robot_pos);
  // TrajectoryPointMsg via_pos;
  // via_pos.pose.position.x = robot_pos.x();
  // via_pos.pose.position.y = robot_pos.y();
  // double cfg_tolerate = config_.obstacles.min_obstacle_dist/2.0;
  // double j = 0;
  // double num_wpts = hypot(via_pos.pose.position.x- goal_pos.x(),via_pos.pose.position.y-goal_pos.y())/cfg_tolerate;
  //  tf::Quaternion q;
  // while(j-num_wpts<1e-6)
  // {
  //   via_pos.pose.position.x = robot_pos.x() + j / num_wpts * (goal_pos.x() - robot_pos.x());
  //   via_pos.pose.position.y = robot_pos.y() + j / num_wpts * (goal_pos.y() - robot_pos.y());
  //   q.setRPY(0,0,robot_pos.theta()+ j / num_wpts * (goal_pos.theta() - robot_pos.theta()));
  //   tf::quaternionTFToMsg(q,via_pos.pose.orientation);
  //   traj.push_back(via_pos);
  //   j++;
  // }
  // for(auto pose:traj)
  // {
    // predict_pose.x()    = pose.pose.position.x;
    // predict_pose.y()    = pose.pose.position.y;
    // tf::quaternionMsgToTF(pose.pose.orientation,q);
    // predict_pose.theta()    = tf::getYaw(q);
    for (ObstContainer::const_iterator obst = obst_vector_.begin(); obst != obst_vector_.end(); ++obst)
    {
      boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);   
      if (!pobst)
        continue;
      double obs_dis = pobst->getMinimumDistance(robot_pos.position(),goal_pos.position());
      //double obs_dis = robot_model_->calculateDistance(predict_pose,pobst.get());
      if(obs_dis < 0.1)
      {
        return false;
      }
    }
  // }

  return true;
}



void teb_planner::setViaPoints(const nav_msgs::Path& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg.poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}
// void teb_planner::interpolatePath(nav_msgs::Path& path)
// {
//   std::vector<geometry_msgs::PoseStamped> temp_path;
//   for (int i = 0; i < static_cast<int>(path.poses.size()-1); i++)
//   {
//     // calculate distance between two consecutive waypoints
//     double x1 = path.poses[i].pose.position.x;
//     double y1 = path.poses[i].pose.position.y;
//     double x2 = path.poses[i+1].pose.position.x;
//     double y2 = path.poses[i+1].pose.position.y;
//     double dist =  hypot(x1-x2, y1-y2);
//     int num_wpts = dist * waypoints_per_meter_;

//     temp_path.push_back(path.poses[i]);
//     geometry_msgs::PoseStamped p = path.poses[i];
//     for (int j = 0; j < num_wpts - 2; j++)
//     {
//       p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
//       p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
//       temp_path.push_back(p);
//     }
//   }

//   // update sequence of poses
//   for (size_t i = 0; i < temp_path.size(); i++)
//     temp_path[i].header.seq = static_cast<int>(i);

//   temp_path.push_back(path.poses.back());
//   path.poses = temp_path;
// }


// bool teb_planner::findGuiWayPath(nav_msgs::Path& path)
// {
//   //构建辅助节点
//   vector<PoseSE2> nodes;
//   double my_config_goal_inflation_x = 1.5;
//   double my_config_goal_inflation_y = 1;
//   double goal_inflation_x_   = my_config_goal_inflation_x;
//   double goal_inflation_y_   = my_config_goal_inflation_y;
//   //create nodes
//   nodes.push_back(PoseSE2(goal_inflation_x_,0,M_PI/2));
//   nodes.push_back(PoseSE2(goal_inflation_x_,-goal_inflation_y_,0));
//   nodes.push_back(PoseSE2(goal_inflation_x_,goal_inflation_y_,0));
//   nodes.push_back(PoseSE2(-goal_inflation_x_,-goal_inflation_y_,0));
//   nodes.push_back(PoseSE2(-goal_inflation_x_,goal_inflation_y_,0));
//   tf::Transform odom_goal;
//   odom_goal.setOrigin(tf::Vector3(endpose_.x(),endpose_.y(),0));
//   tf::Quaternion q(0,0,endpose_.theta());   
//   odom_goal.setRotation(q);
//   tf::Transform goal_node;
//   tf::Transform odom_node;
//   vector<PoseSE2> path_tmp;
//   for(auto itr:nodes)
//   {
//     goal_node.setOrigin(tf::Vector3(itr.x(),itr.y(),0));
//     q.setRPY(0,0,itr.theta());
//     goal_node.setRotation(q);
//     odom_node = odom_goal*goal_node;
//     path_tmp.push_back(PoseSE2(odom_node.getOrigin().getX(),odom_node.getOrigin().getY(),\
//                        odom_node.getRotation().getZ()));
//   }
//   //找到最近的节点
//   int i = 0;
//   double min_dis = 1e6;
//   int min_node;
//   double dis;
//   for(auto itr:path_tmp)
//   {
//     dis = hypot(itr.x()-startpose_.x(),itr.y()-startpose_.y());
//     if (dis<min_dis)
//     {
//       min_dis = dis;
//       min_node = i;
//     }
//     i++;
//   }
//   if(min_node == 0)
//   {
    
//   }
//   geometry_msgs::PoseStamped pos_tmp;

//   path.poses.push_back(pos_tmp);


//   return true;
// }