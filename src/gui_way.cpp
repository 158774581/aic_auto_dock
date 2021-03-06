#include <aic_auto_dock/gui_way.h>

gui_way::gui_way(ros::NodeHandle& nh, ros::NodeHandle& local_nh)
    : nh_(nh), local_nh_(local_nh), as_(new Server(local_nh, "gui_way", boost::bind(&gui_way::start, this, _1), false)),\
    simple_goal_client_(new Client_gui_way("aic_auto_dock_guiway/gui_way", true))
{
  as_->start();

  setFootprint_sub_ = nh_.subscribe("/move_base/local_costmap/footprint", 10, &gui_way::setFootprintCallback, this);
  laser_sub_ = nh_.subscribe("/scan", 10, &gui_way::LaserCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &gui_way::odomCallback, this);
  twist_pub_ = nh_.advertise< geometry_msgs::Twist >("/dock_cmd_vel", 10);
  //  sub_robot_exception_ = nh_.subscribe("/robot_exception", 10, &gui_way::robotExceptionCallback, this);
  marker_pub_CarRadius = local_nh_.advertise< visualization_msgs::Marker >("extra_range", 10);
  marker_pub_ = local_nh_.advertise< visualization_msgs::Marker >("extra_range", 10);
  virtual_path_pub_ = local_nh_.advertise< visualization_msgs::Marker >("virtual_path", 10);

  simple_goal_sub_ = nh_.subscribe("/move_base_simple/goal1", 1, &gui_way::CB_simple_goal,this);
  vector< geometry_msgs::Point > points = makeFootprintFromParams(nh);
  points_ = points;

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

bool gui_way::initDetector(double& percent, tf::Transform frame, double detector_y_coordinate)
{
  int square;
  double radix = 1.2, base = 0.0, scale;
  if (process_.portStepProcessing)
    scale = scale_;
  else
    scale = prepare_scale_;

  /********** 车头正前方 left detector ***********/
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  vector< tf::Transform > foot_leftDetector_frame;
  vector< detector > port_leftDetector;
  square = 0;
  for (int i = 0; i < 100; i++)
  {
    tf::Transform _foot_leftDetector_frame(
        q, tf::Vector3(detector_y_coordinate, delta_detector_spacing_ / 2 + (i * delta_detector_spacing_), 0));
    foot_leftDetector_frame.push_back(_foot_leftDetector_frame);
  }
  for (vector< tf::Transform >::iterator cit = foot_leftDetector_frame.begin(); cit != foot_leftDetector_frame.end();
       cit++)
  {
    //    detector _port_leftDetector = {(frame * (*cit)).getOrigin().getY(), -pow(radix, square), square};
    //    detector _port_leftDetector = {(frame * (*cit)).getOrigin().getY(), -exp(square+1), square};
    detector _port_leftDetector = {
        (frame * (*cit)).getOrigin().getY(),
        (exp(square * scale) - exp(-square * scale)) / (exp(square * scale) + exp(-square * scale)), square};

    port_leftDetector.push_back(_port_leftDetector);
    square++;
  }

  /********** 车头正前方 right detector ***********/
  vector< tf::Transform > foot_rightDetector_frame;
  vector< detector > port_rightDetector;
  square = 0;
  for (int i = 0; i < 100; i++)
  {
    tf::Transform _foot_rightDetector_frame(
        q, tf::Vector3(detector_y_coordinate, -(delta_detector_spacing_ / 2 + (i * delta_detector_spacing_)), 0));
    foot_rightDetector_frame.push_back(_foot_rightDetector_frame);
  }
  for (vector< tf::Transform >::iterator cit = foot_rightDetector_frame.begin(); cit != foot_rightDetector_frame.end();
       cit++)
  {
    //    detector _port_rightDetector = {(frame * (*cit)).getOrigin().getY(), pow(radix, square), square};
    //    detector _port_rightDetector = {(frame * (*cit)).getOrigin().getY(), exp(square + 1), square};
    detector _port_rightDetector = {
        (frame * (*cit)).getOrigin().getY(),
        -(exp(square * scale) - exp(-square * scale)) / (exp(square * scale) + exp(-square * scale)), square};

    port_rightDetector.push_back(_port_rightDetector);
    square++;

    base += _port_rightDetector.weight;
  }

  vector< detector > port_detector;
  port_detector.insert(port_detector.end(), port_leftDetector.begin(), port_leftDetector.end());
  port_detector.insert(port_detector.end(), port_rightDetector.begin(), port_rightDetector.end());

  int j = 0;
  double min_dist = INFINITY;
  for (int i = 0; i < port_detector.size(); i++)
  {
    if (fabs(port_detector[i].dist) < min_dist)
    {
      min_dist = fabs(port_detector[i].dist);
      j = i;
    }
  }

  //  double weight_sum = 1 * ((1.0 - pow(radix, 20)) / (1.0 - radix));
  percent = port_detector[j].weight;
  //    ROS_WARN("percent:%f, j:%d, weight:%f, base:%f", percent, j, port_detector[j].weight, base);

  double allowable_percent = 0.8, allowable_dist;

  for (int i = 0; i < port_rightDetector.size(); i++)
  {
    if (fabs(port_rightDetector[i].weight) > allowable_percent)
    {
      allowable_dist = port_detector[i].num * delta_detector_spacing_;
      break;
    }
  }

  //  ROS_INFO_THROTTLE(1, "percent:%f, dist:%f, allowable_dist:%f", percent, port_detector[j].num *
  //  delta_detector_spacing_, allowable_dist);

  if (percent > allowable_percent)
  {
    ROS_DEBUG_NAMED("guiway", "Warning, percent == 1:%f, dist:%f, allowable_dist:%f", percent,
                    port_detector[j].num * delta_detector_spacing_, allowable_dist);

    percent = allowable_percent;
    return false;
  }
  else if (percent < -allowable_percent)
  {
    ROS_DEBUG_NAMED("guiway", "Warning, percent == -1:%f, dist:%f, allowable_dist:%f", percent,
                    port_detector[j].num * delta_detector_spacing_, allowable_dist);

    percent = -allowable_percent;
    return false;
  }
  return true;
}

/*                                      +(小车）
*                                  0.1 <+(小车）
*         prepareNav_foot_frame +
*                            0.1|
*                          XXXXX|XXXXX  preparePosition_ + 0.1
*           prepare_foot_frame  +
*                                       +(小车）
*                                      <+(小车）
*
*                          XXXXX+XXXXX  preparePosition_
*         prepareNav_foot_frame |
*                            0.1|
*           prepare_foot_frame  +
*
*
*                           X   XXXXXX
*                            X X
*                             X
*/

void gui_way::initFrame(void)
{
  tf::Quaternion q_port_prepare(0.0, 0.0, 0.0);
  port_foot_frame = odom_port_frame_.inverse() * realTime_odom_;

  if (port_foot_frame.getOrigin().getX() - 0.1 > preparePosition_)
    preparePositionNav = port_foot_frame.getOrigin().getX() - 0.1;
  else
    preparePositionNav = preparePosition_;

  tf::Transform port_prepare_frame(q_port_prepare, tf::Vector3(preparePositionNav - 0.1, 0.0, 0.0));
  tf::Transform port_prepareTemp_frame(q_port_prepare, tf::Vector3(preparePositionNav, 0.0, 0.0));

  prepare_foot_frame = port_prepare_frame.inverse() * port_foot_frame;

  tf::Transform prepareTemp_foot_frame = port_prepareTemp_frame.inverse() * port_foot_frame;

  Point start = {(realTime_odom_ * prepareTemp_foot_frame.inverse()).getOrigin().x(),
                 (realTime_odom_ * prepareTemp_foot_frame.inverse()).getOrigin().y()};
  Point end = {realTime_odom_.getOrigin().x(), realTime_odom_.getOrigin().y()};
  Point pt1 = {0, 0};
  Point pt2 = {1, 0};
  prepareNavAngle_ = getAngelOfXaxisM_PI(start, end, pt1, pt2);
  tf::Quaternion q_odom_prepareNav(0, 0, prepareNavAngle_);
  tf::Transform odom_prepareNav_frame_ = realTime_odom_ * prepareTemp_foot_frame.inverse();
  odom_prepareNav_frame_.setRotation(q_odom_prepareNav);
  prepareNav_foot_frame = odom_prepareNav_frame_.inverse() * realTime_odom_;

  br_.sendTransform(
      tf::StampedTransform(realTime_odom_ * port_foot_frame.inverse(), ros::Time::now(), "odom", "odom_port"));
  br_.sendTransform(
      tf::StampedTransform(realTime_odom_ * prepare_foot_frame.inverse(), ros::Time::now(), "odom", "odom_prepare11"));
  br_.sendTransform(tf::StampedTransform(odom_prepareNav_frame_, ros::Time::now(), "odom", "odom_prepareNav11"));
}

void gui_way::workingFrame(void)
{
  tf::Quaternion q_port_prepare(0.0, 0.0, 0.0);
  port_foot_frame = odom_port_frame_.inverse() * realTime_odom_;

  tf::Transform port_prepare_frame(q_port_prepare, tf::Vector3(preparePositionNav - 0.1, 0.0, 0.0));
  tf::Transform port_prepareTemp_frame(q_port_prepare, tf::Vector3(preparePositionNav, 0.0, 0.0));

  prepare_foot_frame = port_prepare_frame.inverse() * port_foot_frame;
  tf::Transform prepareTemp_foot_frame = port_prepareTemp_frame.inverse() * port_foot_frame;

  tf::Quaternion q_odom_prepareNav(0, 0, prepareNavAngle_);
  tf::Transform odom_prepareNav_frame_ = realTime_odom_ * prepareTemp_foot_frame.inverse();
  odom_prepareNav_frame_.setRotation(q_odom_prepareNav);
  prepareNav_foot_frame = odom_prepareNav_frame_.inverse() * realTime_odom_;

  br_.sendTransform(
      tf::StampedTransform(realTime_odom_ * port_foot_frame.inverse(), ros::Time::now(), "odom", "odom_port"));
  br_.sendTransform(
      tf::StampedTransform(realTime_odom_ * prepare_foot_frame.inverse(), ros::Time::now(), "odom", "odom_prepare"));
  br_.sendTransform(tf::StampedTransform(odom_prepareNav_frame_, ros::Time::now(), "odom", "odom_prepareNav"));
}

void gui_way::start(const aic_auto_dock::gui_way2GoalConstPtr& req)
{
  laser_sub_ = nh_.subscribe("/scan", 10, &gui_way::LaserCallback, this);

  cout << endl;
  cout << endl;
  ROS_INFO("gui way accept goal");

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
  ROS_INFO("obstacle_dist_:%f", obstacle_dist_);
  preparePosition_ = req->preparePosition;
  if (req->scale > 0 || req->scale < 0.1)
    scale_ = req->scale;
  else
    scale_ = 0.1;

  /*** 全局环境变量 ***/
  actionlib_status_ = executing;
  motion_status_ = empty;

  /*** 初始化目标点 ***/
  tf::poseMsgToTF(req->pose, odom_port_frame_);

  /*** 本地变量 ***/
  double vel_line = vel_line_, vel_angle = vel_angle_;
  double angleVel_turn = angleVel_turn_;
  double aW_dcc = aW_dcc_, aW_acc = aW_acc_; //旋转时的角加速度
  double initYaw = 0.0;
  bool turnMark = false;
  geometry_msgs::Twist twist;
  bool turnBool = true, slowdownBool = true, arriveturnBool = true;
  ros::Rate rate(20);
  docking_direction direction;
  double yawThrehold;
  double begin_time = ros::Time::now().toSec();
  cleanProcess();

  /*** 初始化target_foot_frame ***/
  initFrame();

  /*** 判断初始步骤 ***/
  initStepProcess(radiusThrehold_);

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

    double dt = ros::Time::now().toSec() - begin_time;
    begin_time = ros::Time::now().toSec();

    pubVirtualPath();

    /*** 获取target_foot_frame ***/
    workingFrame();

    /*** 步骤选择 ***/
    if (process_.process == StepProcess::Process::prepareForceStep)
    {
      if (!process_.prepareNavStepResult)
      {
        ROS_DEBUG_NAMED("guiway", "prepare nav");
        if (!process_.prepareNavStepInit)
        {
          process_.prepareNavStepInit = true;
          turnBool = true;
          slowdownBool = true;
          arriveturnBool = false;
          process_.prepareNavStepProcessing = true;
          back_dist_ = 0.02;
          yawThrehold = yawThrehold_;
          vel_line = prepare_lineVel_;
          vel_angle = prepare_angleVel_;
          /*** 确定行进方向 ***/
          if (direction_ == docking_direction::forward)
          {
            if (prepareNav_foot_frame.inverse().getOrigin().getX() < 0)
              direction = docking_direction::backward;
            else
              direction = direction_;
          }
          else if (direction_ == docking_direction::backward)
          {
            if (prepareNav_foot_frame.inverse().getOrigin().getX() > 0)
              direction = docking_direction::forward;
            else
              direction = direction_;
          }
        }
        target_foot_frame = prepareNav_foot_frame;
      }
      else if (!process_.prepareStepResult)
      {
        ROS_DEBUG_NAMED("guiway", "prepare");
        if (!process_.prepareStepInit)
        {
          process_.prepareStepInit = true;
          direction = direction_;
          turnBool = true;
          slowdownBool = false;
          arriveturnBool = false;
          process_.prepareStepProcessing = true;
          back_dist_ = 0.01;
          yawThrehold = 0.1;
          vel_line = prepare_lineVel_;
          vel_angle = prepare_angleVel_;
        }
        target_foot_frame = prepare_foot_frame;
      }
      else if (!process_.portStepResult)
      {
        ROS_DEBUG_NAMED("guiway", "port");
        if (!process_.portStepInit)
        {
          process_.portStepInit = true;
          direction = direction_;
          turnBool = false;
          slowdownBool = true;
          arriveturnBool = true;
          process_.portStepProcessing = true;
          back_dist_ = req->back_dist;
          yawThrehold = yawThrehold_;
          vel_line = vel_line_;
          vel_angle = vel_angle_;
        }
        target_foot_frame = port_foot_frame;
      }
    }
    else if (process_.process == StepProcess::Process::prepareStep)
    {
      if (!process_.prepareNavStepResult)
      {
        ROS_DEBUG_NAMED("guiway", "prepare nav");
        if (!process_.prepareNavStepInit)
        {
          process_.prepareNavStepInit = true;
          turnBool = true;
          slowdownBool = false;
          arriveturnBool = false;
          process_.prepareNavStepProcessing = true;
          back_dist_ = 0.1;
          yawThrehold = yawThrehold_;
          vel_line = prepare_lineVel_;
          vel_angle = prepare_angleVel_;
          /*** 确定行进方向 ***/
          direction = direction_;
        }
        target_foot_frame = prepareNav_foot_frame;
      }
      else if (!process_.prepareStepResult)
      {
        ROS_DEBUG_NAMED("guiway", "prepare");
        if (!process_.prepareStepInit)
        {
          process_.prepareStepInit = true;
          direction = direction_;
          turnBool = false;
          slowdownBool = false;
          arriveturnBool = false;
          process_.prepareStepProcessing = true;
          back_dist_ = 0.00;
          yawThrehold = 0.1;
          vel_line = prepare_lineVel_;
          vel_angle = prepare_angleVel_;
        }
        target_foot_frame = prepare_foot_frame;
      }
      else if (!process_.portStepResult)
      {
        ROS_DEBUG_NAMED("guiway", "port");
        if (!process_.portStepInit)
        {
          process_.portStepInit = true;
          direction = direction_;
          turnBool = false;
          slowdownBool = true;
          arriveturnBool = true;
          process_.portStepProcessing = true;
          back_dist_ = req->back_dist;
          yawThrehold = yawThrehold_;
          vel_line = vel_line_;
          vel_angle = vel_angle_;
        }
        target_foot_frame = port_foot_frame;
      }
    }
    else if (process_.process == StepProcess::Process::portStep)
    {
      if (!process_.prepareStepResult)
      {
        ROS_DEBUG_NAMED("guiway", "prepare");
        if (!process_.prepareStepInit)
        {
          process_.prepareStepInit = true;
          direction = direction_;
          turnBool = true;
          slowdownBool = false;
          arriveturnBool = false;
          process_.prepareStepProcessing = true;
          back_dist_ = 0.00;
          yawThrehold = 0.1;
          vel_line = prepare_lineVel_;
          vel_angle = prepare_angleVel_;
        }
        target_foot_frame = prepare_foot_frame;
      }
      else if (!process_.portStepResult)
      {
        ROS_DEBUG_NAMED("guiway", "port");
        if (!process_.portStepInit)
        {
          process_.portStepInit = true;
          direction = direction_;
          turnBool = false;
          slowdownBool = true;
          arriveturnBool = true;
          process_.portStepProcessing = true;
          back_dist_ = req->back_dist;
          yawThrehold = yawThrehold_;
          vel_line = vel_line_;
          vel_angle = vel_angle_;
        }
        target_foot_frame = port_foot_frame;
      }
    }

    /*** 获取foot_target_frame的yaw ***/
    double roll, pitch, yaw;
    tf::Matrix3x3(target_foot_frame.inverse().getRotation()).getRPY(roll, pitch, yaw);

    /*** 初始化检测器 ***/
    double detector_y_coordinate =
        (direction == docking_direction::backward) ? -detector_y_coordinate_ : detector_y_coordinate_;
    double percent;
    initDetector(percent, target_foot_frame, detector_y_coordinate);
    percent = (direction == docking_direction::backward) ? -percent : percent; //尾部对接，左右检测器的权重正负号相反

    //    ROS_INFO("percent:%f", percent);

    /*           X           forward      X      backward
    *  foot_port X  port +            +   X   +            +
    *            X       |            |   X   |            |
    *            X       v            v   X   v            v
    *            X                        X
    *            X  foot +-->      <--+   X   +-->      <--+
    *            X                        X
    *  yaw       X       >=0         <0   X   >=0         <0
    *            X                        X
    *            X                        X
    *  percent   X       >=0         <0   X   >=0         <0 */
    double allowable_yaw = fabs(getDiffAngle(fabs(yaw), M_PI_2));
    if (process_.portStepProcessing)
    {
      if ((direction == docking_direction::forward && yaw >= 0 && percent >= 0 && allowable_yaw < 0.3) ||
          (direction == docking_direction::forward && yaw < 0 && percent < 0 && allowable_yaw < 0.3) ||
          (direction == docking_direction::backward && yaw >= 0 && percent < 0 && allowable_yaw < 0.3) ||
          (direction == docking_direction::backward && yaw < 0 && percent >= 0 && allowable_yaw < 0.3)) // 5du
      {
        ROS_INFO("yaw:%f, allowable_yaw:%f, percent:%f", yaw, allowable_yaw, percent);
        percent = 0.0;
      }
    }

    /*** 减速识别特征板 ***/
    //        if (process_.portStepProcessing && fabs(target_foot_frame.getOrigin().x()) <= half_length_ + 0.45)
    //        {
    //          vel_line = 0.01;
    //          vel_angle = 0.15;
    //        }
    //        if (process_.portStepProcessing && fabs(target_foot_frame.getOrigin().x()) <= half_length_ + 0.34)
    //        {
    //          vel_line = req->vel_line;
    //          vel_angle = req->vel_angle;
    //          if ((vel_line == 0) || (vel_angle == 0))
    //          {
    //            vel_line = 0.1;
    //            vel_angle = 0.4;
    //          }
    //        }

    /*** gui way detected ***/
    avoidType_ = AvoidType::RECTANGLE;
    double ang = vel_angle * percent;
    double vel = vel_line * (1 - fabs(percent));
    if (vel == 0.0)
      vel = 0.1;
    twist.angular.z = ang;
    twist.linear.x = (direction == docking_direction::backward) ? -vel : vel;
    //    ROS_INFO("angel vel:%f", twist.angular.z);

    /*** slow down the linear speed ***/
    if ((target_foot_frame.getOrigin().x() <= back_dist_ + 0.1) && slowdownBool)
    {
      if (twist.linear.x > 0.0)
        twist.linear.x = 0.05;
      else if (twist.linear.x < 0.0)
        twist.linear.x = -0.05;
    }

    /*** arrive target position ***/
    if (target_foot_frame.getOrigin().x() <= back_dist_)
    {
      avoidType_ = AvoidType::RECTANGLE;

      double dtYaw1 = fabs(getDiffAngle(yaw, M_PI));
      double dtYaw2 = fabs(getDiffAngle(yaw, 0.0));
      if (direction == docking_direction::forward && dtYaw1 > yawThrehold && arriveturnBool)
      {
        ROS_DEBUG_NAMED("guiway", "arrive forward turn:%f, yawThrehold:%f, dtYaw1:%f", yaw, yawThrehold, dtYaw1);
        twist.linear.x = 0;
        double s_acc = (pow(angleVel_turn, 2) - pow(0.2, 2)) / (2 * aW_acc);
        if (!turnMark)
          initYaw = dtYaw1;
        turnMark = true;
        double diff_s_acc = fabs(getDiffAngle(dtYaw1, initYaw));

        double s2_dcc = (pow(0.05, 2) - pow(angleVel_turn, 2)) / (2 * -aW_dcc);
        double diff_s_dcc = fabs(getDiffAngle(dtYaw1, yawThrehold));
        if (diff_s_dcc > fabs(s2_dcc))
        {
          if (diff_s_acc < s_acc)
          {
            double v = sqrt(pow(0.2, 2) + diff_s_acc * 2 * aW_acc);
            twist.angular.z = (yaw > 0 ? -v : v);
            ROS_DEBUG_NAMED("guiway", "v:%f, dt:%f, dtYaw1:%f, initYaw:%f, diff_s_acc:%f", v, dt, dtYaw1, initYaw,
                            diff_s_acc);
          }
          else
            twist.angular.z = (yaw > 0 ? -angleVel_turn : angleVel_turn);
        }
        else
        {
          double v = sqrt(pow(0.05, 2) - diff_s_dcc * 2 * -aW_dcc);
          twist.angular.z = (yaw > 0 ? -v : v);
          ROS_DEBUG_NAMED("guiway", "v:%f, dt:%f, dtYaw1:%f, diff_s_dcc:%f", v, dt, dtYaw1, diff_s_dcc);
        }
      }
      else if (direction == docking_direction::backward && dtYaw2 > yawThrehold && arriveturnBool)
      {
        ROS_DEBUG_NAMED("guiway", "arrive backward turn:%f, yawThrehold:%f, dtYaw2:%f", yaw, yawThrehold, dtYaw2);
        twist.linear.x = 0;
        double s_acc = (pow(angleVel_turn, 2) - pow(0.2, 2)) / (2 * aW_acc);
        if (!turnMark)
          initYaw = dtYaw2;
        turnMark = true;
        double diff_s_acc = fabs(getDiffAngle(dtYaw2, initYaw));

        double s_dcc = (pow(0.05, 2) - pow(angleVel_turn, 2)) / (2 * -aW_dcc);
        double diff_s_dcc = fabs(getDiffAngle(dtYaw2, yawThrehold));
        if (diff_s_dcc > fabs(s_dcc))
        {
          if (diff_s_acc < s_acc)
          {
            double v = sqrt(pow(0.2, 2) + diff_s_acc * 2 * aW_acc);
            twist.angular.z = (yaw > 0 ? v : -v);
            ROS_DEBUG_NAMED("guiway", "v:%f, dt:%f, dtYaw2:%f, initYaw:%f, diff_s_acc:%f", v, dt, dtYaw2, initYaw,
                            diff_s_acc);
          }
          else
            twist.angular.z = (yaw > 0 ? angleVel_turn : -angleVel_turn);
        }
        else
        {
          double v = sqrt(pow(0.05, 2) - diff_s_dcc * 2 * -aW_dcc);
          twist.angular.z = (yaw > 0 ? v : -v);
          ROS_DEBUG_NAMED("guiway", "v:%f, dt:%f, dtYaw2:%f, diff_s_dcc:%f", v, dt, dtYaw2, diff_s_dcc);
        }
      }
      else
      {
        turnMark = false;
        if (process_.prepareNavStepProcessing)
        {
          process_.prepareNavStepProcessing = false;
          process_.prepareNavStepResult = true;
        }
        if (process_.prepareStepProcessing)
        {
          process_.prepareStepProcessing = false;
          process_.prepareStepResult = true;
        }
        if (process_.portStepProcessing)
        {
          process_.portStepProcessing = false;
          process_.portStepResult = true;
          ROS_INFO("back_dist: %f\ttarget_foot_frame: %f", back_dist_, target_foot_frame.getOrigin().x());
          motion_status_ = success;
          break;
        }
      }
    }

    /***
     * turn first
     ***/
    double dtYaw1 = fabs(getDiffAngle(yaw, M_PI));
    double dtYaw2 = fabs(getDiffAngle(yaw, 0.0));
    if (turnBool && direction == docking_direction::forward && fabs(dtYaw1) > yawThrehold)
    {
      avoidType_ = AvoidType::ROUND;
      ROS_DEBUG_NAMED("guiway", "turn first:%f, dtYaw1:%f", yaw, dtYaw1);
      twist.linear.x = 0;
      double s_acc = (pow(angleVel_turn, 2) - pow(0.05, 2)) / (2 * aW_acc);
      if (!turnMark)
        initYaw = dtYaw1;
      turnMark = true;
      double diff_s_acc = fabs(getDiffAngle(dtYaw1, initYaw));

      double s_dcc = (pow(0.05, 2) - pow(angleVel_turn, 2)) / (2 * -aW_dcc);
      double diff_s_dcc = fabs(getDiffAngle(fabs(dtYaw1), yawThrehold));
      if (diff_s_dcc > fabs(s_dcc))
      {
        if (diff_s_acc < s_acc)
        {
          double v = sqrt(pow(0.05, 2) + diff_s_acc * 2 * aW_acc);
          twist.angular.z = (yaw > 0 ? -v : v);
          ROS_DEBUG_NAMED("guiway", "v:%f, dt:%f, dtYaw1:%f, initYaw:%f, diff_s_acc:%f", v, dt, dtYaw1, initYaw,
                          diff_s_acc);
        }
        else
          twist.angular.z = (yaw > 0 ? -angleVel_turn : angleVel_turn);
      }
      else
      {
        double v = sqrt(pow(0.05, 2) - diff_s_dcc * 2 * -aW_dcc);
        twist.angular.z = (yaw > 0 ? -v : v);
        ROS_DEBUG_NAMED("guiway", "v:%f, dt:%f, dtYaw1:%f", v, dt, dtYaw1);
      }
      ROS_DEBUG_NAMED("guiway", "S_acc:%f, S_dcc:%f", s_acc, s_dcc);
    }
    else if (turnBool && direction == docking_direction::backward && dtYaw2 > yawThrehold)
    {
      avoidType_ = AvoidType::ROUND;
      ROS_DEBUG_NAMED("guiway", "turn first:%f, dtYaw2:%f", yaw, dtYaw2);
      twist.linear.x = 0;
      double s_acc = (pow(angleVel_turn, 2) - pow(0.05, 2)) / (2 * aW_acc);
      if (!turnMark)
        initYaw = dtYaw2;
      turnMark = true;
      double diff_s_acc = fabs(getDiffAngle(dtYaw2, initYaw));

      double s_dcc = (pow(0.05, 2) - pow(angleVel_turn, 2)) / (2 * -aW_dcc);
      double diff_s_dcc = fabs(getDiffAngle(dtYaw2, yawThrehold));
      if (diff_s_dcc > fabs(s_dcc))
      {
        if (diff_s_acc < s_acc)
        {
          double v = sqrt(pow(0.05, 2) + diff_s_acc * 2 * aW_acc);
          twist.angular.z = (yaw > 0 ? v : -v);
          ROS_DEBUG_NAMED("guiway", "v:%f, dt:%f, dtYaw2:%f, initYaw:%f, diff_s_acc:%f", v, dt, dtYaw2, initYaw,
                          diff_s_acc);
        }
        else
          twist.angular.z = (yaw > 0 ? angleVel_turn : -angleVel_turn);
      }
      else
      {
        double v = sqrt(pow(0.05, 2) - diff_s_dcc * 2 * -aW_dcc);
        twist.angular.z = (yaw > 0 ? v : -v);
        ROS_DEBUG_NAMED("guiway", "v:%f, dt:%f, dtYaw2:%f", v, dt, dtYaw2);
      }
      ROS_DEBUG_NAMED("guiway", "S_acc:%f, S_dcc:%f", s_acc, s_dcc);
    }
    else
    {
      if (turnBool)
        turnMark = false;
      turnBool = false;
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

      ROS_ERROR_THROTTLE(3, "find obstacle! fabs(yaw):%f", fabs(yaw));
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

void gui_way::initStepProcess(const double& allowable_dist)
{
  /***
  *                 +   +
  *                 |   |
  *                 |   |
  *   prepareNavMark|   |prepareNavMark
  *                 |   |
  *                 |   |
  *                 |   |
  *      -- -- -- --+   +-- -- -- -- linePosition
  *                 |   |
  * preparePosition_|   |
  *          *******|   |*******
  *                 |   |
  *                 +---+
  *
  *           prepareForeceNavMark
  *
  *             X      XXXXXXXXXX
  *              X    X
  *               X  X
  *                 X
  ***/

  /*** 判断初始步骤 ***/
  double linePosition = preparePosition_ + 0.1;
  bool prepareForceNavMark = (port_foot_frame.getOrigin().getX() < linePosition) &&
                             !((fabs(port_foot_frame.getOrigin().getY()) < allowable_dist) &&
                               (port_foot_frame.getOrigin().getX() < linePosition) &&
                               (port_foot_frame.getOrigin().getX() > linePosition - allowable_dist));
  bool prepareNavMark = (port_foot_frame.getOrigin().getX() > linePosition) &&
                        (fabs(port_foot_frame.getOrigin().getY()) > allowable_dist);
  if (prepareForceNavMark)
  {
    ROS_WARN("prepare force nav");
    process_.process = StepProcess::Process::prepareForceStep;
  }
  else if (prepareNavMark)
  {
    ROS_WARN("prepare nav");
    process_.process = StepProcess::Process::prepareStep;
  }
  else
  {
    ROS_WARN("port, port_foot_frame.getOrigin().getY():%f", port_foot_frame.getOrigin().getY());
    process_.process = StepProcess::Process::portStep;
  }
}

void gui_way::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Vector3 vec(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  realTime_odom_.setOrigin(vec);
  realTime_odom_.setRotation(q);
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
   accept_laserScan = true;
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
          //accept_laserScan = true;

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
          //accept_laserScan = true;

          return;
        }
      }
    }

    danger_mark_ = false;
    //accept_laserScan = true;
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
        initFrame();
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
  if (accept_robotInfo_ == false)
  {
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
  if (accept_robotInfo_ == false)
  {
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

void gui_way::cleanProcess()
{
  process_.prepareNavStepProcessing = false;
  process_.prepareNavStepResult = false;
  process_.prepareNavStepInit = false;

  process_.prepareStepProcessing = false;
  process_.prepareStepResult = false;
  process_.prepareStepInit = false;

  process_.portStepProcessing = false;
  process_.portStepResult = false;
  process_.portStepInit = false;
}

void gui_way::CB_simple_goal(const geometry_msgs::PoseStampedConstPtr& msg)
{
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
