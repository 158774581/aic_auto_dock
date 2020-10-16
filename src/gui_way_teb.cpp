/*
  author :7
  date   :2020.9.18
*/
#include <aic_auto_dock/gui_way_teb.h>

//gui way 

gui_way::gui_way(ros::NodeHandle& nh, ros::NodeHandle& local_nh)
    : nh_(nh), local_nh_(local_nh), as_(new Server(local_nh, "gui_way", boost::bind(&gui_way::start, this, _1), false)),
    planner_(new teb_planner(local_nh)),simple_goal_client_(new Client_gui_way("aic_auto_dock_guiway/gui_way", true))
{
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
  local_nh_.param< std::string >("laser_frame_name", laser_frame_name_, "laser_link");
  loadParamFromYaml();

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
  //debug
  geometry_msgs::Point debug_p;
  debug_p.x = 0.5;debug_p.y = 0.25;  points_.push_back(debug_p);
  debug_p.x = -0.5;debug_p.y = 0.25;  points_.push_back(debug_p);
  debug_p.x = -0.5;debug_p.y = -0.25;  points_.push_back(debug_p);
  debug_p.x = 0.5;debug_p.y = -0.25;  points_.push_back(debug_p);
  //end debug

  //teb
  simple_goal_sub_ = nh.subscribe("/move_base_simple/goal1", 1, &gui_way::CB_simple_goal,this);
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
  //accept_robotInfo_ = true;

  try
  {
    listerner_.waitForTransform("base_footprint", laser_frame_name_, ros::Time(0), ros::Duration(10.0));
    listerner_.lookupTransform("base_footprint", laser_frame_name_, ros::Time(0), foot_laser_frame_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
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
  planner_->getTebConfig().robot.max_vel_x = vel_line_;
  planner_->getTebConfig().robot.max_vel_theta = vel_angle_;
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
    Laserhander(scan_msg_);

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
      ROS_ERROR("can't find transform 'odom','base_footprint'");
    }
    tf::Transform odom_goal_frame(odom_port_frame_);
    q.setRPY(0,0,0);
    tf::Transform goal_nav_frame(q);
    target_foot_frame = (odom_foot_frame.inverse()*odom_port_frame_).inverse();
    double cfg_y = 0.2;
    double cfg_yaw = 0.2;
    //get global path
    nav_msgs::Path path_raw;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped via_p;
    tf::quaternionTFToMsg(q,via_p.pose.orientation);
    double x = target_foot_frame.getOrigin().getX();
    double y = target_foot_frame.getOrigin().getY();
    double goal_yaw = tf::getYaw(target_foot_frame.getRotation());
    if (req->type == aic_auto_dock::gui_way2Goal::STRAIGHT)
    {
      goal_yaw = goal_yaw>0? goal_yaw-M_PI:goal_yaw+M_PI;
    }
    if (x > preparePosition_||fabs(y)>cfg_y||fabs(goal_yaw)>cfg_yaw)
    {
      via_p.pose.position.x = 0;  
      via_p.pose.position.y = 0;  
      path_raw.poses.push_back(via_p);
      via_p.pose.position.x = 0.3*preparePosition_;  
      via_p.pose.position.y = 0;  
      path_raw.poses.push_back(via_p);
      via_p.pose.position.x = 0.6*preparePosition_;  
      via_p.pose.position.y = 0;  
      path_raw.poses.push_back(via_p);
      via_p.pose.position.x = preparePosition_;  
      via_p.pose.position.y = 0;  
      path_raw.poses.push_back(via_p);
      via_p.pose.position.x = x;  
      via_p.pose.position.y = y;  
      path_raw.poses.push_back(via_p);
    }
    else
    {
      double aux_point[] = {0,0,  x*0.3,0,  x*0.6,0,  x*0.8,0,  x,y};
      via_p.pose.position.x = 0;  
      via_p.pose.position.y = 0;  
      path_raw.poses.push_back(via_p);
      via_p.pose.position.x = 0.3*x;  
      via_p.pose.position.y = 0;  
      path_raw.poses.push_back(via_p);
      via_p.pose.position.x = 0.6*x;  
      via_p.pose.position.y = 0;  
      path_raw.poses.push_back(via_p);
      via_p.pose.position.x = 0.8*x;  
      via_p.pose.position.y = 0;  
      path_raw.poses.push_back(via_p);
      via_p.pose.position.x = x;  
      via_p.pose.position.y = y;  
      path_raw.poses.push_back(via_p);
    }
    planner_->interpolatePath(odom_port_frame_,path_raw,path);
    goal_pos.x() = path.poses.back().pose.position.x;
    goal_pos.y() = path.poses.back().pose.position.y;
    goal_pos.theta() = tf::getYaw(path.poses.back().pose.orientation);
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
      motion_status_ = failed;
      ROS_INFO("Unable to plan a valid path !!!");
      break;
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
      motion_status_ = failed;
      break;
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
  scan_msg_ = msg;
  accept_laserScan = true;
}
void gui_way::Laserhander(const sensor_msgs::LaserScan& msg)
{
  tf::Quaternion q;
  planner_->clearObstacle();
  //U型的车库障碍物
  double cfg_x = 0.8;
  double cfg_y = 0.5;
  //add obstacles
  q.setRPY(0,0,0);
  tf::Transform goal_obs_frame(q,tf::Vector3(cfg_x,-cfg_y,0));
  tf::Transform goal_obs1_frame(q,tf::Vector3(-cfg_x,-cfg_y,0));
  tf::Transform odom_obs_frame = odom_port_frame_*goal_obs_frame;
  tf::Transform odom_obs1_frame = odom_port_frame_*goal_obs1_frame;
  planner_->setLineObstacle(odom_obs_frame.getOrigin().getX(), odom_obs_frame.getOrigin().getY(),\
                            odom_obs1_frame.getOrigin().getX(), odom_obs1_frame.getOrigin().getY());

  // goal_obs_frame.setOrigin(tf::Vector3(-cfg_x,-cfg_y,0));
  // goal_obs1_frame.setOrigin(tf::Vector3(-cfg_x,cfg_y,0));
  // odom_obs_frame = odom_port_frame_*goal_obs_frame;
  // odom_obs1_frame = odom_port_frame_*goal_obs1_frame;
  // planner_->setLineObstacle(odom_obs_frame.getOrigin().getX(), odom_obs_frame.getOrigin().getY(),\
  //                           odom_obs1_frame.getOrigin().getX(), odom_obs1_frame.getOrigin().getY());

  goal_obs_frame.setOrigin(tf::Vector3(-cfg_x,cfg_y,0));
  goal_obs1_frame.setOrigin(tf::Vector3(cfg_x,cfg_y,0));
  odom_obs_frame = odom_port_frame_*goal_obs_frame;
  odom_obs1_frame = odom_port_frame_*goal_obs1_frame;
  planner_->setLineObstacle(odom_obs_frame.getOrigin().getX(), odom_obs_frame.getOrigin().getY(),\
                            odom_obs1_frame.getOrigin().getX(), odom_obs1_frame.getOrigin().getY());

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

      q.setRPY(0, 0, 0);
      tf::Transform laser_pt_frame(q, tf::Vector3(x, y, 0));
      tf::Transform foot_pt_frame = foot_laser_frame_ * laser_pt_frame;
      tf::Transform odom_pt_frame = realTime_odom_*foot_pt_frame;
      //将距离小于1.5m的障碍物添加到TEB规划避障中
      double cfg_dis = 1;
      if (hypot(foot_pt_frame.getOrigin().getX(),foot_pt_frame.getOrigin().getY())<cfg_dis)
      {
         planner_->setPointObstacle(odom_pt_frame.getOrigin().getX(),odom_pt_frame.getOrigin().getY());
      }

      if (avoidType_ == AvoidType::ROUND)
      {
        bool boolcarBodyPadding = pnpoly::polygon_contains(
            points_turn_polygon_padding_, foot_pt_frame.getOrigin().getX(), foot_pt_frame.getOrigin().getY());
        bool boolcarBody = pnpoly::polygon_contains(points_polygon_, foot_pt_frame.getOrigin().getX(),
                                                    foot_pt_frame.getOrigin().getY());

        if (dist_laser > msg.range_min && dist_laser < msg.range_max && (boolcarBody || boolcarBodyPadding))
        {
          danger_mark_ = true;
          // accept_laserScan = true;

          return;
        }
      }
      else
      {
        bool boolcarBodyPadding = pnpoly::polygon_contains(
            points_straight_polygon_padding_, foot_pt_frame.getOrigin().getX(), foot_pt_frame.getOrigin().getY());
        bool boolcarBody = pnpoly::polygon_contains(points_polygon_, foot_pt_frame.getOrigin().getX(),
                                                    foot_pt_frame.getOrigin().getY());

        if (dist_laser > msg.range_min && dist_laser < msg.range_max && (boolcarBody || boolcarBodyPadding))
        {
          danger_mark_ = true;
          // accept_laserScan = true;

          return;
        }
      }
    }

    danger_mark_ = false;
    // accept_laserScan = true;
  }
  else
    ROS_INFO_ONCE("move avoid node cannot get the robot information, temporarily shut down obstacle avoidance function!");
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
  if (accept_robotInfo_==false)
  {
    ROS_INFO_ONCE("function 'pubMarkerCarStraightSquare()' error:no robot info !");
    return;
  }
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
  if (accept_robotInfo_==false)
  {
    ROS_INFO_ONCE("function 'pubMarkerCarTurnSquare()' error:no robot info !");
    return;
  }
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
  guiway_goal.obstacle_dist = 0.5;//fabs(half_length_);
  guiway_goal.preparePosition = 2;
  simple_goal_client_->waitForServer();
  ROS_INFO("wait server succeed");

  simple_goal_client_->sendGoal(guiway_goal);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "guiWay");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  gui_way guiWay(nh, nh_local);

  ros::MultiThreadedSpinner spin;
  spin.spin();
}