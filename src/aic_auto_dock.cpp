#include "aic_auto_dock/aic_auto_dock.h"

autodock_interface::autodock_interface(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
    : nh_(nh), nh_local_(nh_local),
      server_(new Server(nh, "aic_auto_dock", boost::bind(&autodock_interface::Start, this, _1), false)),
      client_gui_way_(new Client_gui_way("aic_auto_dock_guiway/gui_way", true)),
      v_shape_extra_(new VShapeExtra(nh, nh_local)), parallel_line_extra_(new ParallelLineExtra(nh, nh_local)),
      elevator_extra_(new ElevatorExtra(nh, nh_local)),
      line_extractor_(new line_extraction::LineExtractionROS(nh, nh_local)),
      client_move_avoid_(new Client_move_avoid("aic_move_avoid", true)), VShapeData_(new autodock_data),
      ParallelLineData_(new autodock_data), ElevatorData_(new autodock_data),
      /*7_add*/ client_auto_dock_(new client_auto_dock(nh,"aic_auto_dock",true))
{
  client_gui_way_->waitForServer();
  ROS_INFO("dock interface actionlib client: link gui_way successful!");

  client_move_avoid_->waitForServer();
  ROS_INFO("dock interface actionlib client: link move_avoid successful!");

  initparam();

  setFootprint_sub_ =
      nh_.subscribe("/move_base/local_costmap/set_footprint", 10, &autodock_interface::setFootprintCallback, this);
  robot_info_sub_ = nh_.subscribe("/robot_info", 10, &autodock_interface::robotInfoCallback, this);
  laser_sub_ = nh_.subscribe("/autodock/scan", 10, &autodock_interface::LaserCallback, this);

  status_code_pub_ = nh_.advertise< std_msgs::UInt64 >("/status_code", 10);
  twist_pub_ = nh_.advertise< geometry_msgs::Twist >("/dock_cmd_vel", 10);
  marker_pub_ = nh_local_.advertise< visualization_msgs::Marker >("avoiding_range", 10);

  gui_way_thread_ = new boost::thread(boost::bind(&autodock_interface::GuiWayThread, this));
  move_avoid_thread_ = new boost::thread(boost::bind(&autodock_interface::MoveAvoidThread, this));
  //7_add
  auto_dock_thread_ = new boost::thread(boost::bind(&autodock_interface::AutoDockThread, this));
  odom_sub_ = nh_.subscribe("/odom", 1, &autodock_interface::odomCallback, this);
  ROS_WARN("aic auto dock wait move_base_simple/goal...");
  simple_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &autodock_interface::CB_simple_goal,this);
  move_base_cancle_pub_ =  nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
  //end
  server_->start();
  ROS_INFO("dock interface actionlib: Init successful!");
}

autodock_interface::~autodock_interface()
{
  if (line_extractor_ != nullptr)
    delete line_extractor_;

  if (server_ != nullptr)
    delete server_;

  if (client_gui_way_ != nullptr)
    delete client_gui_way_;

  if (client_move_avoid_ != nullptr)
    delete client_move_avoid_;

  if (v_shape_extra_ != nullptr)
    delete v_shape_extra_;

  if (parallel_line_extra_ != nullptr)
    delete parallel_line_extra_;

  if (elevator_extra_ != nullptr)
    delete elevator_extra_;

  //  if (Data_ != nullptr)
  //    delete Data_;

  if (VShapeData_ != nullptr)
    delete VShapeData_;

  if (ParallelLineData_ != nullptr)
    delete ParallelLineData_;

  if (ElevatorData_ != nullptr)
    delete ElevatorData_;

  if (gui_way_thread_ != nullptr)
  {
    gui_way_thread_->interrupt();
    gui_way_thread_->join();
    delete gui_way_thread_;
  }
  if (move_avoid_thread_ != nullptr)
  {
    move_avoid_thread_->interrupt();
    move_avoid_thread_->join();
    delete move_avoid_thread_;
  }
}

bool autodock_interface::initparam()
{
  nh_local_.param< string >("scan_frame", scan_frame_, "laser_link");

  /***/
  try
  {
    listerner_.waitForTransform("base_footprint", scan_frame_, ros::Time(0), ros::Duration(10.0));
    listerner_.lookupTransform("base_footprint", scan_frame_, ros::Time(0), foot_laser_frame_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  /***/
  vector< geometry_msgs::Point > points = makeFootprintFromParams(nh_);
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
  carRadius_ = sqrt(pow(half_length_, 2) + pow(half_width_, 2)) + 0.03;

  /*** 避障区域 ***/
  points_polygon_.points.clear();
  points_polygon_padding_.points.clear();
  for (vector< geometry_msgs::Point >::iterator cit = points.begin(); cit != points.end(); cit++)
  {
    geometry_msgs::Point32 pt_padding;
    pt_padding.x = cit->x + (cit->x >= 0 ? 0.1 : -0.1);
    pt_padding.y = cit->y + (cit->y >= 0 ? 0.1 : -0.1);

    geometry_msgs::Point32 pt1;
    pt1.x = cit->x;
    pt1.y = cit->y;
    points_polygon_.points.push_back(pt1);
    points_polygon_padding_.points.push_back(pt_padding);
  }
  return true;
}

bool autodock_interface::initgoal(const aic_msgs::AutoDock2GoalConstPtr& goal)
{
  /*** param ***/
  AicExtractionInterface interface;
  AicExtractionPolygon polygon;

  /*** init goal ***/
  interface.tag = goal->tag_no;
  double detal_length;
  if (goal->board_shape == aic_msgs::AutoDock2Goal::VL_SHAPE)
  {
    ROS_INFO("vl shape board type");
    Data_ = VShapeData_;
    extra_pt_ = v_shape_extra_;
    loadParamFromYaml(goal->tag_no, "v_shape_param");
    interface.name = "v_shape_param";
    pfun = &autodock_interface::guiwayMotionWithVshapeCalibration;
    detal_length = half_length_ + 0.3;
  }
  else if (goal->board_shape == aic_msgs::AutoDock2Goal::PARALLEL_SHAPE)
  {
    ROS_INFO("parallel shape board type");
    Data_ = ParallelLineData_;
    extra_pt_ = parallel_line_extra_;
    loadParamFromYaml(goal->tag_no, "parallel_lines_param");
    interface.name = "parallel_lines_param";
    pfun = &autodock_interface::guiwayMotionWithParallelCalibration;
    detal_length = 0.0;
  }
  else if (goal->board_shape == aic_msgs::AutoDock2Goal::ELEVATOR_SHAPE)
  {
    ROS_INFO("elevator shape board type");
    Data_ = ElevatorData_;
    extra_pt_ = elevator_extra_;
    loadParamFromYaml(goal->tag_no, "elevator_param");
    interface.name = "elevator_param";
    pfun = &autodock_interface::guiwayMotionWithElevatorCalibration;
    detal_length = 0.0;
  }
  else
  {
    ROS_ERROR("error board type");
    return false;
  }

  /*** init direction ***/
  if (goal->dock_direction == aic_msgs::AutoDock2Goal::BACKWARD)
    Data_->dock_direction_ = docking_direction::backward;
  else if (goal->dock_direction == aic_msgs::AutoDock2Goal::FRONTWARD)
    Data_->dock_direction_ = docking_direction::forward;
  else
  {
    Data_->dock_direction_ = docking_direction::still;
    ROS_ERROR("error docking direction");
    return false;
  }

  /*** init extra range ***/
  geometry_msgs::Point32 pt1, pt2, pt3, pt4;
  pt1.x = 0.0;
  pt1.y = -(0.5 + half_width_ + recognize_deltaY_);
  pt2.x = preparePosition_ + detal_length + recognize_deltaX_;
  pt2.y = -(0.5 + half_width_ + recognize_deltaY_);
  pt3.x = preparePosition_ + detal_length + recognize_deltaX_;
  pt3.y = 0.5 + half_width_ + recognize_deltaY_;
  pt4.x = 0.0;
  pt4.y = 0.5 + half_width_ + recognize_deltaY_;
  if (Data_->dock_direction_ == docking_direction::backward)
  {
    pt2.x = -pt2.x;
    pt3.x = -pt3.x;
  }
  geometry_msgs::Point32 arr_range[] = {pt1, pt2, pt3, pt4};
  vector< geometry_msgs::Point32 > vec_range(arr_range, arr_range + 4);
  polygon.extra_polygon.points = vec_range;
  polygon.frame_id = "base_footprint";

  /*** input param ***/
  extra_pt_->setLineExtractor(line_extractor_);
  extra_pt_->setInterface(interface);
  extra_pt_->setDirection(Data_->dock_direction_);
  if (!extra_pt_->setExtraPolygon(polygon))
    return false;

  /***
   * init
   ***/
  actionlib_status_ = executing;
  actionlib_last_status_ = actionlib_status_;
  feedback_.err_msg = 0;
  feedback_.status = aic_msgs::AutoDock2Feedback::EXECUTING;
  feedback_.feedback = aic_msgs::AutoDock2Feedback::LEGAL_GOAL;
  server_->publishFeedback(feedback_);

  Data_->transform_status_ = success;
  Data_->motion_status_ = success;

  return true;
}

void autodock_interface::Start(const aic_msgs::AutoDock2GoalConstPtr& goal)
{
  cout << endl;
  cout << endl;
  ROS_INFO("autodock interface actionlib server: Accept goal!");

  if (!initgoal(goal))
  {
    ROS_ERROR("docking interface actionlib server: docking init goal failed");
    results_.err_msg = 0;
    results_.result = aic_msgs::AutoDock2Result::FAILED;
    results_.feature_direction = aic_msgs::AutoDock2Result::EMPTY;
    server_->setAborted(results_);
    return;
  }

  /***
   * run
   ***/
  if (goal->type == aic_msgs::AutoDock2Goal::ENTER)
  {
    ROS_INFO("docking interface actionlib server: enter status");
    run(goal);
    extra_pt_->setUnSubscribe();
  }
  else if (goal->type == aic_msgs::AutoDock2Goal::LEAVE)
  {
    ROS_INFO("docking interface actionlib server: leave status");
    leave_run(goal);
    extra_pt_->setUnSubscribe();
  }
  else if (goal->type == aic_msgs::AutoDock2Goal::RESET)
  {
    ROS_INFO("docking interface actionlib server: reset status");

    extra_pt_->setUnSubscribe();

    if (VShapeData_ != nullptr)
      delete VShapeData_;
    if (ParallelLineData_ != nullptr)
      delete ParallelLineData_;
    if (ElevatorData_ != nullptr)
      delete ElevatorData_;

    VShapeData_ = new autodock_data;
    ParallelLineData_ = new autodock_data;
    ElevatorData_ = new autodock_data;

    results_.err_msg = 0;
    results_.result = aic_msgs::AutoDock2Result::SUCCESS;
    results_.feature_direction = aic_msgs::AutoDock2Result::EMPTY;
    server_->setSucceeded(results_);
    return;
  }
  else
  {
    ROS_ERROR("docking interface actionlib server: Accept goal error! type error! Please start first!");

    extra_pt_->setUnSubscribe();

    results_.err_msg = 0;
    results_.result = aic_msgs::AutoDock2Result::FAILED;
    results_.feature_direction = aic_msgs::AutoDock2Result::EMPTY;
    server_->setAborted(results_);
    return;
  }

  ros::Duration(1).sleep();

  /***
   * result
   ***/
  if (Data_->motion_status_ == success && Data_->transform_status_ == success)
  {
    ROS_INFO("docking interface actionlib server: Success");
    results_.err_msg = 0;
    results_.result = aic_msgs::AutoDock2Result::SUCCESS;
    results_.feature_direction = aic_msgs::AutoDock2Result::EMPTY;
    server_->setSucceeded(results_);
  }
  else if ((Data_->transform_status_ == canclegoal) || (Data_->motion_status_ == canclegoal))
  {
    ROS_INFO("docking interface actionlib server: cancle");
    results_.err_msg = 0;
    results_.result = aic_msgs::AutoDock2Result::CANCLE;
    results_.feature_direction = aic_msgs::AutoDock2Result::EMPTY;
    server_->setPreempted(results_);
    return;
  }
  else
  {
    ROS_INFO("docking interface actionlib server: failed");
    server_->setAborted(results_);
  }
}

void autodock_interface::run(const aic_msgs::AutoDock2GoalConstPtr& goal)
{
  /***
 * start
 ***/
  if ((number_lidar_ == 2) || (number_lidar_ == 1))
  {
    /***
     * step 1
     ***/
    Data_->module_data.currentStep = ModuleData::RecognizeStep;
    Data_->module_data.current_status = false;
    Data_->module_data.remaining_distance = 0.0;

    feedback_.err_msg = SYS_SECONDARY_LOCALIZATION_RECOGNIZE_START;
    feedback_.status = aic_msgs::AutoDock2Feedback::EXECUTING;
    feedback_.feedback = aic_msgs::AutoDock2Feedback::LEGAL_GOAL;
    server_->publishFeedback(feedback_);
    status_code_.data = SYS_SECONDARY_LOCALIZATION_RECOGNIZE_START;
    status_code_pub_.publish(status_code_);
    ROS_INFO("SYS_SECONDARY_LOCALIZATION_RECOGNIZE_START");

    docking_direction moving = docking_direction::still;
    geometry_msgs::Pose precise_pose;
    extra_pt_->setRoughRecgonizeMark(false);
    if (!recognize(moving, precise_pose))
    {
      if (Data_->transform_status_ == canclegoal)
        return;
      ROS_WARN("docking interface actionlib server: far first still Recognition fail");

      bool recognizeMark = false;
      recognizeMark = recognizeLogic(precise_pose, false);
      if(!recognizeMark)
      {
        if (Data_->transform_status_ == canclegoal)
          return;
        recognizeMark = recognizeLogic(precise_pose, true);
        if(!recognizeMark)
        {
          if (Data_->transform_status_ == canclegoal)
            return;
        }
      }

      if (!recognizeMark)
      {
        status_code_.data = SYS_SECONDARY_LOCALIZATION_RECOGNIZE_FAILED;
        status_code_pub_.publish(status_code_);
        results_.err_msg = SYS_SECONDARY_LOCALIZATION_RECOGNIZE_FAILED;
        results_.result = aic_msgs::AutoDock2Result::FAILED;
        return;
      }
    }
    Data_->module_data.current_status = true;

    /***
     * step 2
     ***/
    Data_->module_data.currentStep = ModuleData::DockingStep;
    Data_->module_data.current_status = false;
    Data_->module_data.remaining_distance = 0.0;
    feedback_.err_msg = SYS_SECONDARY_LOCALIZATION_DOCKING_START;
    feedback_.status = aic_msgs::AutoDock2Feedback::EXECUTING;
    feedback_.feedback = aic_msgs::AutoDock2Feedback::LEGAL_GOAL;
    server_->publishFeedback(feedback_);
    status_code_.data = SYS_SECONDARY_LOCALIZATION_DOCKING_START;
    status_code_pub_.publish(status_code_);
    ROS_INFO("SYS_SECONDARY_LOCALIZATION_DOCKING_START");

    aic_auto_dock::gui_way2Goal guiway_goal;
    int gui_way_direction;
    if (Data_->dock_direction_ == docking_direction::backward) // 0:前进  1:后退
      gui_way_direction = aic_auto_dock::gui_way2Goal::BACK;
    else
      gui_way_direction = aic_auto_dock::gui_way2Goal::STRAIGHT;

    guiway_goal.type = gui_way_direction;
    guiway_goal.pose = precise_pose;
    guiway_goal.vel_line = guiway_vel_line_;
    guiway_goal.vel_angle = guiway_vel_angle_;
    guiway_goal.back_dist = fabs(half_length_) + delta_backDist_;
    guiway_goal.obstacle_dist = fabs(half_length_) + obstacle_dist_;
    guiway_goal.preparePosition = preparePosition_;
    if (!(this->*pfun)(guiway_goal))
    {
      if (Data_->motion_status_ == failed)
      {
        status_code_.data = SYS_SECONDARY_LOCALIZATION_DOCKING_FAILED;
        status_code_pub_.publish(status_code_);
        results_.err_msg = SYS_SECONDARY_LOCALIZATION_DOCKING_FAILED;
        results_.result = aic_msgs::AutoDock2Result::FAILED;
        ROS_WARN("docking interface actionlib server: Motion fail");
      }
      else if (Data_->motion_status_ == timeout)
      {
        status_code_.data = SYS_SECONDARY_LOCALIZATION_DOCKING_TIMEOUT;
        status_code_pub_.publish(status_code_);
        results_.err_msg = SYS_SECONDARY_LOCALIZATION_DOCKING_TIMEOUT;
        results_.result = aic_msgs::AutoDock2Result::FAILED;
        ROS_WARN("docking interface actionlib server: Motion timeout");
      }

      Data_->module_data.remaining_distance = fabs(guiWay_remaining_distance_ - half_length_);
      return;
    }
    Data_->module_data.current_status = true;
  }
  //  else if (number_lidar_ == 1)
  //  {
  //    Data_->transform_status_ = failed;
  //    Data_->motion_status_ = failed;
  //    ROS_WARN("autodock interface node is suitable for one lidar");
  //    return;
  //  }
  else
  {
    Data_->transform_status_ = failed;
    Data_->motion_status_ = failed;
    ROS_WARN("autodock interface node cannot get the robot information, return false!");
    return;
  }
}

bool autodock_interface::recognizeLogic(geometry_msgs::Pose& precise_pose, bool rough_mark)
{
  ROS_INFO("recognizeLogic mark:%d", rough_mark);
  bool recognizeMark = false;
  for (int i = 0; i < 2; i++)
  {
    docking_direction moving = docking_direction::rotate;
    extra_pt_->setRoughRecgonizeMark(rough_mark);
    if (!recognize(moving, precise_pose))
    {
      if (Data_->transform_status_ == canclegoal)
        return false;
      ROS_WARN("docking interface actionlib server: recognizeLogic rotate Recognition fail:%d", i);

      recognizeMark = false;
    }
    else
    {
      ros::Duration(0.5).sleep();
      docking_direction moving = docking_direction::still;
      extra_pt_->setRoughRecgonizeMark(rough_mark);
      if (!recognize(moving, precise_pose))
      {
        if (Data_->transform_status_ == canclegoal)
          return false;
        ROS_WARN("docking interface actionlib server: recognizeLogic still Recognition fail:%d", i);

        recognizeMark = false;
      }
      else
      {
        recognizeMark = true;
        break;
      }
    }
  }
  return recognizeMark;
}

bool autodock_interface::recognize(const docking_direction& moving, geometry_msgs::Pose& precise_pose)
{
  vector< tf::Transform > vec_precise_pose_far;
  geometry_msgs::Pose pose_temp;

  double beginTime = ros::Time::now().toSec(), leftTime;
  double featureMatchingTimeout = (moving == docking_direction::still) ? 4 : featureMatchingTimeout_;

  extra_pt_->setFarDetection(true);
  while (ros::ok())
  {
    geometry_msgs::Pose pose_far;
    if (actionlib_last_status_ == executing)
    {
      if (extra_pt_->run(pose_temp, pose_far))
      {
        tf::Transform pose_frame;
        tf::poseMsgToTF(pose_far, pose_frame);
        //7_add
        tf::Transform nav_port_frame = (nav_position_.inverse())*pose_frame;
        if(fabs(nav_port_frame.getOrigin().getY())<0.5&&nav_port_frame.getOrigin().getX()>0.0)//port位置合理
        {
          vec_precise_pose_far.push_back(pose_frame);
          pose_temp = pose_far;
        }
        //7_del
        //vec_precise_pose_far.push_back(pose_frame);
        //pose_temp = pose_far;

        if (moving == docking_direction::still && vec_precise_pose_far.size() >= 5)
        {
          tf::Transform precise_pose_frame = recognize_extraction::extra_filter(vec_precise_pose_far);
          tf::poseTFToMsg(precise_pose_frame, precise_pose);
          Data_->transform_status_ = success;
          ROS_INFO("far still extra average");
          return true;
        }
        else if (moving != docking_direction::still)
        {
          precise_pose = pose_far;
          Data_->transform_status_ = success;
          ROS_INFO("far moving extra once");
          return true;
        }
      }
    }

    if (!recognizeThread(featureMatchingTimeout, Data_->transform_status_, beginTime, leftTime))
    {
      geometry_msgs::Twist twist;
      twist.angular.z = 0.0;
      twist.linear.x = 0.0;
      twist_pub_.publish(twist);

      return false;
    }

    recognizeMovingStrategy(moving, featureMatchingTimeout, beginTime);
  }
}

bool autodock_interface::recognizeMovingStrategy(const docking_direction& moving, const double& featureMatchingTimeout,
                                                 double& beginTime)
{
  /***
   * moving strategy
   ***/
  if (actionlib_status_ == executing)
  {
    geometry_msgs::Twist twist;
    if (moving == docking_direction::rotate)
    {
      if (ros::Time::now().toSec() - beginTime < featureMatchingTimeout / 4)
      {
        twist.angular.z = -0.1;
        twist.linear.x = 0.0;
      }
      if (ros::Time::now().toSec() - beginTime > featureMatchingTimeout / 4 &&
          ros::Time::now().toSec() - beginTime < (featureMatchingTimeout / 4) * 3)
      {
        twist.angular.z = 0.1;
        twist.linear.x = 0.0;
      }
      if (ros::Time::now().toSec() - beginTime > (featureMatchingTimeout / 4) * 3 &&
          ros::Time::now().toSec() - beginTime < featureMatchingTimeout)
      {
        twist.angular.z = -0.1;
        twist.linear.x = 0.0;
      }
    }
    else if (moving == docking_direction::forward)
    {
      twist.angular.z = 0.0;
      twist.linear.x = 0.1;

      if (ros::Time::now().toSec() - beginTime > 4)
      {
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
      }
    }
    else if (moving == docking_direction::backward)
    {
      twist.angular.z = 0.0;
      twist.linear.x = -0.1;

      if (ros::Time::now().toSec() - beginTime > 4)
      {
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
      }
    }

    if (moving != docking_direction::still && danger_mark_)
    {
      twist.angular.z = 0.0;
      twist.linear.x = 0.0;
    }

    if (moving != docking_direction::still)
    {
      twist_pub_.publish(twist);
      pubMarkerCarStraightSquare();
    }
  }
}

bool autodock_interface::recognizeThread(const double& featureMatchingTimeout, status& thread_status, double& beginTime,
                                         double& leftTime)
{
  if (!goalAccept())
  {
    ROS_WARN("dock interface actionlib server: thread Cancle goal");
    thread_status = canclegoal;
    return false;
  }

  /***
   * pause and resum
   ***/
  if (actionlib_last_status_ == executing && actionlib_status_ == paused)
  {
    actionlib_last_status_ = actionlib_status_;
    leftTime = ros::Time::now().toSec() - beginTime;
  }
  else if (actionlib_last_status_ == paused && actionlib_status_ == executing)
  {
    actionlib_last_status_ = actionlib_status_;
    beginTime = ros::Time::now().toSec() - leftTime;
  }

  /***
   * regconize time out
   ***/
  if (ros::Time::now().toSec() - beginTime > featureMatchingTimeout && featureMatchingTimeout > 0 &&
      actionlib_status_ == executing)
  {
    ROS_WARN("dock interface:feature matching timeout");
    thread_status = timeout;
    return false;
  }
  return true;
}

bool autodock_interface::guiwayMotionWithVshapeCalibration(aic_auto_dock::gui_way2Goal& goal)
{
  cleanProcess();

  vector< tf::Transform > vec_precise_pose1, vec_precise_pose2, vec_precise_pose3, vec_precise_pose,
      vec_precise_pose_temp;
  geometry_msgs::Pose temp_pose = goal.pose, pose;

  bool update_guiway = false, update_mark = false;

  goal.scale = scale_;
  client_gui_way_->sendGoal(goal, boost::bind(&autodock_interface::getGuiWayCallback, this, _1, _2),
                            Client_gui_way::SimpleActiveCallback(),
                            boost::bind(&autodock_interface::gui_way_feedbackCB, this, _1));
  ros::Duration(0.1).sleep();
  boost::unique_lock< boost::recursive_mutex > lock(gui_way_mutex_);
  run_gui_way_Thread_ = true;
  gui_way_cond_.notify_one();
  lock.unlock();

  ros::Rate rate(rate_);
  double beginTime = ros::Time::now().toSec();
  int temp_size = 0;
  bool a = true;

  extra_pt_->setRoughRecgonizeMark(false);
  extra_pt_->setFarDetection(false);

  while (ros::ok())
  {
    rate.sleep();

    if ((actionlib_status_ == executing) && a)
    {
      AicExtractionPolygon polygon;
      initPolygon(temp_pose, polygon, 0.5);
      if (!extra_pt_->setExtraPolygon(polygon))
        return false;

      if (extra_pt_->run(temp_pose, pose))
      {
        temp_pose = pose;
        tf::Transform pose_frame;
        tf::poseMsgToTF(pose, pose_frame);

        vec_precise_pose_temp.push_back(pose_frame);
      }

      if (vec_precise_pose_temp.size() > average_times_ + 1)
        vec_precise_pose_temp.clear();
    }

    if (process_.portStepProcessing && a)
    {
      if (guiWayFB_remaining_distance_ < half_length_ + stop_recognizeDist_)
      {
        a = false;
      }

      if (vec_precise_pose_temp.size() > average_times_ && a)
      {
        ROS_INFO("precise");
        ROS_INFO("vec_precise_pose_temp.size():%d, guiWayFB_remaining_distance_:%f", vec_precise_pose_temp.size(),
                 guiWayFB_remaining_distance_);

        tf::Transform precise_pose_frame;
        precise_pose_frame = recognize_extraction::extra_filter(vec_precise_pose_temp);

        geometry_msgs::Pose precise_pose;
        tf::poseTFToMsg(precise_pose_frame, precise_pose);

        br_.sendTransform(
            tf::StampedTransform(precise_pose_frame, ros::Time::now(), "odom", "average_new_Middle_frame"));

        double roll, pitch, yaw;
        tf::Matrix3x3(precise_pose_frame.getRotation()).getRPY(roll, pitch, yaw);
        ROS_WARN("x:%f, y:%f, yaw:%f", precise_pose_frame.getOrigin().x(), precise_pose_frame.getOrigin().y(), yaw);

        goal.type = aic_auto_dock::gui_way2Goal::INITPORT;
        goal.pose = precise_pose;
        if (!((gui_way_thread_status_ == success) || (gui_way_thread_status_ == failed)))
          client_gui_way_->sendGoal(goal, boost::bind(&autodock_interface::getGuiWayCallback, this, _1, _2),
                                    Client_gui_way::SimpleActiveCallback(),
                                    boost::bind(&autodock_interface::gui_way_feedbackCB, this, _1));
        vec_precise_pose_temp.clear();
      }
    }

    lock.lock();

    if (gui_way_thread_status_ == success)
    {
      lock.unlock();
      gui_way_thread_status_ = empty;
      ROS_WARN_NAMED("gui way thread", "gui way thread, success");
      return true;
    }
    else if (gui_way_thread_status_ == failed)
    {
      lock.unlock();
      gui_way_thread_status_ = empty;
      ROS_WARN_NAMED("gui way thread", "gui way thread, failed");
      return false;
    }
    lock.unlock();
  }
}

bool autodock_interface::guiwayMotionWithParallelCalibration(aic_auto_dock::gui_way2Goal& goal)
{
  cleanProcess();

  vector< tf::Transform > vec_precise_pose_temp;
  geometry_msgs::Pose temp_pose = goal.pose, pose;
  //7_add
  goal.port_length = 1.0;
  actionlib_msgs::GoalID  cancle_goal;
  move_base_cancle_pub_.publish(cancle_goal);
  ROS_INFO("cancle move_base/goal");
  //goal.port_width = (parallel_line_extra_->lines_min_gap_+parallel_line_extra_->lines_max_gap_)/2.0;
  goal.scale = scale_;
  client_gui_way_->sendGoal(goal, boost::bind(&autodock_interface::getGuiWayCallback, this, _1, _2),
                            Client_gui_way::SimpleActiveCallback(),
                            boost::bind(&autodock_interface::gui_way_feedbackCB, this, _1));
  ros::Duration(0.1).sleep();
  boost::unique_lock< boost::recursive_mutex > lock(gui_way_mutex_);
  run_gui_way_Thread_ = true;
  gui_way_cond_.notify_one();
  lock.unlock();

  ros::Rate rate(rate_);
  double beginTime = ros::Time::now().toSec();
  int temp_size = 0;
  bool a = true;

  extra_pt_->setRoughRecgonizeMark(false);
  extra_pt_->setFarDetection(false);

  while (ros::ok())
  {
    rate.sleep();

    //    if (process_.portStepProcessing && (guiWayFB_remaining_distance_ < preparePosition_ - 0.05))
    if ((actionlib_status_ == executing) && a)
    {
      AicExtractionPolygon polygon;
      initPolygon(temp_pose, polygon, 0.5);
      if (!extra_pt_->setExtraPolygon(polygon))
        return false;

      if (extra_pt_->run(temp_pose, pose))
      {
        temp_pose = pose;
        tf::Transform pose_frame;
        tf::poseMsgToTF(pose, pose_frame);

        vec_precise_pose_temp.push_back(pose_frame);
      }

      if (vec_precise_pose_temp.size() > average_times_ + 8)
        vec_precise_pose_temp.clear();
    }

    if (process_.portStepProcessing && a)
    {
      if (guiWayFB_remaining_distance_ < half_length_ + stop_recognizeDist_)
      {
        a = false;
      }

      if (vec_precise_pose_temp.size() > average_times_ && a)
      {
        ROS_INFO("precise");
        ROS_INFO("vec_precise_pose_temp.size():%d, guiWayFB_remaining_distance_:%f", vec_precise_pose_temp.size(),
                 guiWayFB_remaining_distance_);

        tf::Transform precise_pose_frame;
        precise_pose_frame = recognize_extraction::extra_filter(vec_precise_pose_temp);

        geometry_msgs::Pose precise_pose;
        tf::poseTFToMsg(precise_pose_frame, precise_pose);

        br_.sendTransform(
            tf::StampedTransform(precise_pose_frame, ros::Time::now(), "odom", "average_new_Middle_frame"));

        double roll, pitch, yaw;
        tf::Matrix3x3(precise_pose_frame.getRotation()).getRPY(roll, pitch, yaw);
        ROS_WARN("x:%f, y:%f, yaw:%f", precise_pose_frame.getOrigin().x(), precise_pose_frame.getOrigin().y(), yaw);

        goal.type = aic_auto_dock::gui_way2Goal::INITPORT;
        goal.pose = precise_pose;
        if (!((gui_way_thread_status_ == success) || (gui_way_thread_status_ == failed)))
          client_gui_way_->sendGoal(goal, boost::bind(&autodock_interface::getGuiWayCallback, this, _1, _2),
                                    Client_gui_way::SimpleActiveCallback(),
                                    boost::bind(&autodock_interface::gui_way_feedbackCB, this, _1));
        vec_precise_pose_temp.clear();
      }
    }

    lock.lock();

    if (gui_way_thread_status_ == success)
    {
      lock.unlock();
      gui_way_thread_status_ = empty;
      ROS_WARN_NAMED("gui way thread", "gui way thread, success");
      return true;
    }
    else if (gui_way_thread_status_ == failed)
    {
      lock.unlock();
      gui_way_thread_status_ = empty;
      ROS_WARN_NAMED("gui way thread", "gui way thread, failed");
      return false;
    }
    lock.unlock();
  }
}

bool autodock_interface::guiwayMotionWithElevatorCalibration(aic_auto_dock::gui_way2Goal& goal)
{
  cleanProcess();

  vector< tf::Transform > vec_precise_pose_temp;
  geometry_msgs::Pose temp_pose = goal.pose, pose;

  goal.scale = scale_;
  client_gui_way_->sendGoal(goal, boost::bind(&autodock_interface::getGuiWayCallback, this, _1, _2),
                            Client_gui_way::SimpleActiveCallback(),
                            boost::bind(&autodock_interface::gui_way_feedbackCB, this, _1));
  ros::Duration(0.1).sleep();
  boost::unique_lock< boost::recursive_mutex > lock(gui_way_mutex_);
  run_gui_way_Thread_ = true;
  gui_way_cond_.notify_one();
  lock.unlock();

  ros::Rate rate(rate_);
  double beginTime = ros::Time::now().toSec();
  int temp_size = 0;
  bool a = true;

  extra_pt_->setRoughRecgonizeMark(false);
  extra_pt_->setFarDetection(false);

  while (ros::ok())
  {
    rate.sleep();

    //    if (process_.portStepProcessing && (guiWayFB_remaining_distance_ < preparePosition_ - 0.05))
    if ((actionlib_status_ == executing) && a)
    {
      AicExtractionPolygon polygon;
      initPolygon(temp_pose, polygon, 0.5);
      if (!extra_pt_->setExtraPolygon(polygon))
        return false;

      if (extra_pt_->run(temp_pose, pose))
      {
        temp_pose = pose;
        tf::Transform pose_frame;
        tf::poseMsgToTF(pose, pose_frame);

        vec_precise_pose_temp.push_back(pose_frame);
      }

      if (vec_precise_pose_temp.size() > average_times_ + 3)
        vec_precise_pose_temp.clear();
    }

    if (process_.portStepProcessing && a)
    {
      if (guiWayFB_remaining_distance_ < half_length_ + stop_recognizeDist_)
      {
        a = false;
      }

      if (vec_precise_pose_temp.size() > average_times_ && a)
      {
        ROS_INFO("precise");
        ROS_INFO("vec_precise_pose_temp.size():%d, guiWayFB_remaining_distance_:%f", vec_precise_pose_temp.size(),
                 guiWayFB_remaining_distance_);

        tf::Transform precise_pose_frame;
        precise_pose_frame = recognize_extraction::extra_filter(vec_precise_pose_temp);

        geometry_msgs::Pose precise_pose;
        tf::poseTFToMsg(precise_pose_frame, precise_pose);

        br_.sendTransform(
            tf::StampedTransform(precise_pose_frame, ros::Time::now(), "odom", "average_new_Middle_frame"));

        double roll, pitch, yaw;
        tf::Matrix3x3(precise_pose_frame.getRotation()).getRPY(roll, pitch, yaw);
        ROS_WARN("x:%f, y:%f, yaw:%f", precise_pose_frame.getOrigin().x(), precise_pose_frame.getOrigin().y(), yaw);

        goal.type = aic_auto_dock::gui_way2Goal::INITPORT;
        goal.pose = precise_pose;
        if (!((gui_way_thread_status_ == success) || (gui_way_thread_status_ == failed)))
          client_gui_way_->sendGoal(goal, boost::bind(&autodock_interface::getGuiWayCallback, this, _1, _2),
                                    Client_gui_way::SimpleActiveCallback(),
                                    boost::bind(&autodock_interface::gui_way_feedbackCB, this, _1));
        vec_precise_pose_temp.clear();
      }
    }

    lock.lock();

    if (gui_way_thread_status_ == success)
    {
      lock.unlock();
      gui_way_thread_status_ = empty;
      ROS_WARN_NAMED("gui way thread", "gui way thread, success");
      return true;
    }
    else if (gui_way_thread_status_ == failed)
    {
      lock.unlock();
      gui_way_thread_status_ = empty;
      ROS_WARN_NAMED("gui way thread", "gui way thread, failed");
      return false;
    }
    lock.unlock();
  }
}

void autodock_interface::initPolygon(const geometry_msgs::Pose& temp_pose, AicExtractionPolygon& polygon,
                                     double delta_y)
{
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  tf::Transform pose_pt1_frame(q, tf::Vector3(preparePosition_, half_width_ + delta_y + recognize_deltaY_, 0.0));
  tf::Transform pose_pt2_frame(q, tf::Vector3(preparePosition_, -half_width_ - delta_y - recognize_deltaY_, 0.0));
  tf::Transform pose_pt3_frame(q, tf::Vector3(-0.3, -half_width_ - delta_y - recognize_deltaY_, 0.0));
  tf::Transform pose_pt4_frame(q, tf::Vector3(-0.3, half_width_ + delta_y + recognize_deltaY_, 0.0));

  tf::Transform odom_pose;
  tf::poseMsgToTF(temp_pose, odom_pose);

  tf::Transform odom_pt1_frame = odom_pose * pose_pt1_frame;
  tf::Transform odom_pt2_frame = odom_pose * pose_pt2_frame;
  tf::Transform odom_pt3_frame = odom_pose * pose_pt3_frame;
  tf::Transform odom_pt4_frame = odom_pose * pose_pt4_frame;

  geometry_msgs::Point32 pt1, pt2, pt3, pt4;
  pt1.x = odom_pt1_frame.getOrigin().getX();
  pt1.y = odom_pt1_frame.getOrigin().getY();
  pt2.x = odom_pt2_frame.getOrigin().getX();
  pt2.y = odom_pt2_frame.getOrigin().getY();
  pt3.x = odom_pt3_frame.getOrigin().getX();
  pt3.y = odom_pt3_frame.getOrigin().getY();
  pt4.x = odom_pt4_frame.getOrigin().getX();
  pt4.y = odom_pt4_frame.getOrigin().getY();

  geometry_msgs::Point32 arr_range[] = {pt1, pt2, pt3, pt4};
  vector< geometry_msgs::Point32 > vec_range(arr_range, arr_range + 4);
  polygon.extra_polygon.points = vec_range;
  polygon.frame_id = "odom";
}

void autodock_interface::GuiWayThread()
{
  ros::NodeHandle n;
  boost::unique_lock< boost::recursive_mutex > lock(gui_way_mutex_);
  ros::Rate rate(10);
  status cancleMark = empty;
  double beginTime = ros::Time::now().toSec();
  double leftTime;

  while (n.ok())
  {
    while (!run_gui_way_Thread_)
    {
      // if we should not be running the planner then suspend this thread
      ROS_INFO("gui way thread, thread is suspending");
      gui_way_cond_.wait(lock);

      cancleMark = empty;
      beginTime = ros::Time::now().toSec();
    }
    lock.unlock();

    rate.sleep();
    if (!goalAccept())
    {
      if (cancleMark != canclegoal)
      {
        cancleMark = canclegoal;
        client_gui_way_->cancelGoal();
      }
    }

    /***
     * pause and resum
     ***/

    aic_auto_dock::gui_way2Goal goal;
    if (actionlib_last_status_ == executing && actionlib_status_ == paused)
    {
      actionlib_last_status_ = actionlib_status_;
      leftTime = ros::Time::now().toSec() - beginTime;
      ROS_INFO("gui way thread, paused");
      goal.type = aic_auto_dock::gui_way2Goal::PAUSE;
      client_gui_way_->sendGoal(goal, boost::bind(&autodock_interface::getGuiWayCallback, this, _1, _2),
                                Client_gui_way::SimpleActiveCallback(),
                                boost::bind(&autodock_interface::gui_way_feedbackCB, this, _1));
    }
    else if (actionlib_last_status_ == paused && actionlib_status_ == executing)
    {
      actionlib_last_status_ = actionlib_status_;
      beginTime = ros::Time::now().toSec() - leftTime;
      ROS_INFO("gui way thread, resum");
      goal.type = aic_auto_dock::gui_way2Goal::RESUM;
      client_gui_way_->sendGoal(goal, boost::bind(&autodock_interface::getGuiWayCallback, this, _1, _2),
                                Client_gui_way::SimpleActiveCallback(),
                                boost::bind(&autodock_interface::gui_way_feedbackCB, this, _1));
    }

    /***
     * result
     ***/
    if (client_gui_way_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && run_gui_way_Thread_)
    {
      lock.lock();
      ROS_WARN("gui way thread, plan thread success");
      Data_->motion_status_ = success;
      gui_way_thread_status_ = success;
      run_gui_way_Thread_ = false;
      lock.unlock();
    }
    else if (((client_gui_way_->getState() == actionlib::SimpleClientGoalState::ABORTED) ||
              (client_gui_way_->getState() == actionlib::SimpleClientGoalState::PREEMPTED)) &&
             run_gui_way_Thread_)
    {
      lock.lock();
      ROS_WARN("gui way thread, plan thread failed");
      Data_->motion_status_ = failed;
      gui_way_thread_status_ = failed;
      run_gui_way_Thread_ = false;

      if (cancleMark == canclegoal)
      {
        ROS_WARN("dock interface actionlib server: Cancle goal");
        Data_->motion_status_ = canclegoal;
      }
      else if (cancleMark == timeout)
      {
        Data_->motion_status_ = timeout;
        gui_way_thread_status_ = failed;
        run_gui_way_Thread_ = false;
      }

      lock.unlock();
    }

    /***
     * timeout
     ***/
    if (ros::Time::now().toSec() - beginTime > stepControlTimeout_ && stepControlTimeout_ > 0 &&
        actionlib_status_ == executing)
    {
      ROS_WARN("gui way thread, dock interface guiway control timeout");
      client_gui_way_->cancelGoal();

      cancleMark = timeout;
    }
    lock.lock();
  }
}

void autodock_interface::getGuiWayCallback(const actionlib::SimpleClientGoalState& state,
                                           const aic_auto_dock::gui_way2ResultConstPtr& result)
{
  if (result->result != aic_auto_dock::gui_way2Result::INITFAILED)
  {
    if (result->remaining_distance >= 0)
      guiWay_remaining_distance_ = result->remaining_distance;
    else
      guiWay_remaining_distance_ = 0.0;
  }
  ROS_INFO("gui way remaining distance:%f", guiWay_remaining_distance_);
}

void autodock_interface::gui_way_feedbackCB(const aic_auto_dock::gui_way2FeedbackConstPtr& msg)
{
  if (msg->feedback == aic_auto_dock::gui_way2Feedback::OBSTACLE_AVOIDING)
  {
    aic_msgs::AutoDock2Feedback feedback;
    feedback.status = aic_msgs::AutoDock2Feedback::EXECUTING;
    feedback.err_msg = SYS_NAV_LOCAL_PLANNER_OBSTACLES_AHEAD;
    server_->publishFeedback(feedback);
    status_code_.data = SYS_NAV_LOCAL_PLANNER_OBSTACLES_AHEAD;
    status_code_pub_.publish(status_code_);
  }
  else if (msg->feedback == aic_auto_dock::gui_way2Feedback::AVOID_SUCCESS)
  {
    aic_msgs::AutoDock2Feedback feedback;
    feedback.status = aic_msgs::AutoDock2Feedback::EXECUTING;
    feedback.err_msg = SYS_NAV_LOCAL_PLANNER_AVOID_SUCCESS;
    server_->publishFeedback(feedback);
    status_code_.data = SYS_NAV_LOCAL_PLANNER_AVOID_SUCCESS;
    status_code_pub_.publish(status_code_);
  }
  else if (msg->feedback == aic_auto_dock::gui_way2Feedback::STEP_PROCESS)
  {
    if (msg->step_process == aic_auto_dock::gui_way2Feedback::PREPARE_NAV_STEP)
      process_.prepareNavStepProcessing = true;
    else if (msg->step_process == aic_auto_dock::gui_way2Feedback::PREPARE_STEP)
      process_.prepareStepProcessing = true;
    else if (msg->step_process == aic_auto_dock::gui_way2Feedback::PORT_STEP)
      process_.portStepProcessing = true;

    guiWayFB_remaining_distance_ = msg->remaining_distance;
  }

  if (msg->status == aic_auto_dock::gui_way2Feedback::EXECUTING)
    guiway_status_ = executing;
  else if (msg->status == aic_auto_dock::gui_way2Feedback::PAUSE)
    guiway_status_ = paused;
}

void autodock_interface::cleanProcess()
{
  process_.prepareNavStepProcessing = false;
  process_.prepareNavStepResult = false;

  process_.prepareStepProcessing = false;
  process_.prepareStepResult = false;

  process_.portStepProcessing = false;
  process_.portStepResult = false;
}

bool autodock_interface::lineStepMotion(double dist, double speed, aic_msgs::OdomCtrolAvoidObstacleGoal control,
                                        bool BoolAvoid)
{
  control.arg_type = 0;
  control.arg = dist;
  control.speed = speed;
  control.useMoveAvoid = BoolAvoid ? 0 : 1;
  client_move_avoid_->sendGoal(control, Client_move_avoid::SimpleDoneCallback(),
                               Client_move_avoid::SimpleActiveCallback(),
                               boost::bind(&autodock_interface::move_avoid_feedbackCB, this, _1));
  ros::Duration(0.1).sleep();
  boost::unique_lock< boost::recursive_mutex > lock(move_avoid_mutex_);
  run_move_avoid_Thread_ = true;
  move_avoid_cond_.notify_one();
  ros::Rate rate(25);
  lock.unlock();
  while (ros::ok())
  {
    rate.sleep();

    lock.lock();

    if (move_avoid_thread_status_ == success)
    {
      lock.unlock();
      aic_msgs::OdomCtrolAvoidObstacleResultConstPtr result = client_move_avoid_->getResult();
      moveAvoid_remaining_distance_ = fabs(result->remaining_distance);

      move_avoid_thread_status_ = empty;
      ROS_INFO("move avoid thread, success remaining dist:%f", moveAvoid_remaining_distance_);
      return true;
    }
    else if (move_avoid_thread_status_ == failed)
    {
      lock.unlock();
      aic_msgs::OdomCtrolAvoidObstacleResultConstPtr result = client_move_avoid_->getResult();
      moveAvoid_remaining_distance_ = fabs(result->remaining_distance);

      move_avoid_thread_status_ = empty;
      ROS_WARN("move avoid thread, failed remaining dist:%f", moveAvoid_remaining_distance_);
      return false;
    }
    lock.unlock();
  }
}

bool autodock_interface::rotationStepMotion(double angle, double speed, aic_msgs::OdomCtrolAvoidObstacleGoal control,
                                            bool BoolAvoid)
{
  control.arg_type = 1;
  control.arg = angle;
  control.speed = speed;
  control.useMoveAvoid = BoolAvoid ? 0 : 1;
  client_move_avoid_->sendGoal(control, Client_move_avoid::SimpleDoneCallback(),
                               Client_move_avoid::SimpleActiveCallback(),
                               boost::bind(&autodock_interface::move_avoid_feedbackCB, this, _1));
  ros::Duration(0.1).sleep();
  boost::unique_lock< boost::recursive_mutex > lock(move_avoid_mutex_);
  run_move_avoid_Thread_ = true;
  move_avoid_cond_.notify_one();
  ros::Rate rate(25);
  lock.unlock();
  while (ros::ok())
  {
    rate.sleep();

    lock.lock();

    if (move_avoid_thread_status_ == success)
    {
      lock.unlock();
      aic_msgs::OdomCtrolAvoidObstacleResultConstPtr result = client_move_avoid_->getResult();
      moveAvoid_remaining_distance_ = fabs(result->remaining_distance);
      move_avoid_thread_status_ = empty;
      ROS_INFO("move avoid thread, success:%f", moveAvoid_remaining_distance_);
      return true;
    }
    else if (move_avoid_thread_status_ == failed)
    {
      lock.unlock();
      aic_msgs::OdomCtrolAvoidObstacleResultConstPtr result = client_move_avoid_->getResult();
      moveAvoid_remaining_distance_ = fabs(result->remaining_distance);
      move_avoid_thread_status_ = empty;
      ROS_WARN("move avoid thread, failed:%f", moveAvoid_remaining_distance_);
      return false;
    }
    lock.unlock();
  }
}

void autodock_interface::MoveAvoidThread()
{
  ros::NodeHandle n;
  boost::unique_lock< boost::recursive_mutex > lock(move_avoid_mutex_);
  ros::Rate rate(25);
  double beginTime = ros::Time::now().toSec();
  double leftTime;
  status cancleMark = empty;
  while (n.ok())
  {
    while (!run_move_avoid_Thread_)
    {
      cancleMark = empty;

      // if we should not be running the planner then suspend this thread
      ROS_INFO("move avoid thread, thread is suspending");
      move_avoid_cond_.wait(lock);
      beginTime = ros::Time::now().toSec();
    }
    lock.unlock();

    rate.sleep();
    if (!goalAccept())
    {
      if (cancleMark != canclegoal)
      {
        cancleMark = canclegoal;
        client_move_avoid_->cancelGoal();
        //      move_avoid_thread_status_ = failed;
        //      run_move_avoid_Thread_ = false;
      }
    }

    /***
     * pause and resum
     ***/
    aic_msgs::OdomCtrolAvoidObstacleGoal control;
    if (actionlib_last_status_ == executing && actionlib_status_ == paused)
    {
      actionlib_last_status_ = actionlib_status_;
      leftTime = ros::Time::now().toSec() - beginTime;
      ROS_INFO("move avoid thread, paused");
      control.arg_type = aic_msgs::OdomCtrolAvoidObstacleGoal::PAUSE;
      client_move_avoid_->sendGoal(control, Client_move_avoid::SimpleDoneCallback(),
                                   Client_move_avoid::SimpleActiveCallback(),
                                   boost::bind(&autodock_interface::move_avoid_feedbackCB, this, _1));
    }
    else if (actionlib_last_status_ == paused && actionlib_status_ == executing)
    {
      actionlib_last_status_ = actionlib_status_;
      beginTime = ros::Time::now().toSec() - leftTime;
      ROS_INFO("move avoid thread, resum");
      control.arg_type = aic_msgs::OdomCtrolAvoidObstacleGoal::RESUM;
      client_move_avoid_->sendGoal(control, Client_move_avoid::SimpleDoneCallback(),
                                   Client_move_avoid::SimpleActiveCallback(),
                                   boost::bind(&autodock_interface::move_avoid_feedbackCB, this, _1));
    }

    /***
     * result
     ***/
    if (client_move_avoid_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && run_move_avoid_Thread_)
    {
      lock.lock();
      ROS_INFO("move avoid thread, plan thread success");
      Data_->motion_status_ = success;
      move_avoid_thread_status_ = success;
      run_move_avoid_Thread_ = false;
      lock.unlock();
    }
    else if (((client_move_avoid_->getState() == actionlib::SimpleClientGoalState::ABORTED) ||
              (client_move_avoid_->getState() == actionlib::SimpleClientGoalState::PREEMPTED)) &&
             run_move_avoid_Thread_)
    {
      lock.lock();
      ROS_WARN("move avoid thread, plan thread failed");
      Data_->motion_status_ = failed;
      move_avoid_thread_status_ = failed;
      run_move_avoid_Thread_ = false;

      if (cancleMark == canclegoal)
      {
        ROS_WARN("dock interface actionlib server: Cancle goal");
        Data_->motion_status_ = canclegoal;
      }
      else if (cancleMark == timeout)
      {
        Data_->motion_status_ = timeout;
        move_avoid_thread_status_ = failed;
        run_move_avoid_Thread_ = false;
      }

      lock.unlock();
    }

    /***
     * timeout
     ***/
    if (ros::Time::now().toSec() - beginTime > stepControlTimeout_ && stepControlTimeout_ > 0 &&
        actionlib_status_ == executing)
    {
      ROS_WARN("move avoid thread, dock interface line control timeout");

      client_move_avoid_->cancelGoal();
      cancleMark = timeout;
    }

    lock.lock();
  }
}

void autodock_interface::move_avoid_feedbackCB(const aic_msgs::OdomCtrolAvoidObstacleFeedbackConstPtr& msg)
{
  if (msg->feedback == aic_msgs::OdomCtrolAvoidObstacleFeedback::OBSTACLE_AVOIDING)
  {
    aic_msgs::AutoDock2Feedback feedback;
    feedback.err_msg = SYS_NAV_LOCAL_PLANNER_OBSTACLES_AHEAD;
    feedback.status = aic_msgs::AutoDock2Feedback::EXECUTING;
    server_->publishFeedback(feedback);
    status_code_.data = SYS_NAV_LOCAL_PLANNER_OBSTACLES_AHEAD;
    status_code_pub_.publish(status_code_);
  }
  else if (msg->feedback == aic_msgs::OdomCtrolAvoidObstacleFeedback::AVOID_SUCCESS)
  {
    aic_msgs::AutoDock2Feedback feedback;
    feedback.err_msg = SYS_NAV_LOCAL_PLANNER_AVOID_SUCCESS;
    feedback.status = aic_msgs::AutoDock2Feedback::EXECUTING;
    server_->publishFeedback(feedback);
    status_code_.data = SYS_NAV_LOCAL_PLANNER_AVOID_SUCCESS;
    status_code_pub_.publish(status_code_);
  }
}

bool autodock_interface::loadParamFromYaml(std::string tag, std::string name)
{
  ROS_INFO("docking tag no:%s, name:%s", tag.c_str(), name.c_str());

  vector< std::string > no_tag;
  vector< double > featureMatchingTimeout, stepControlTimeout, guiway_vel_line, guiway_vel_angle, preparePosition,
      delta_backDist, stop_recognizeDist, obstacle_dist, getout_vel, scale, rate, average_times, recognize_deltaX,
      recognize_deltaY;
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
          if (behavior_list[i].hasMember("no_tag") && behavior_list[i].hasMember("featureMatchingTimeout") &&
              behavior_list[i].hasMember("stepControlTimeout") && behavior_list[i].hasMember("guiway_vel_line") &&
              behavior_list[i].hasMember("guiway_vel_angle") && behavior_list[i].hasMember("preparePosition") &&
              behavior_list[i].hasMember("delta_backDist") && behavior_list[i].hasMember("stop_recognizeDist") &&
              behavior_list[i].hasMember("obstacle_dist") && behavior_list[i].hasMember("getout_vel") &&
              behavior_list[i].hasMember("scale") && behavior_list[i].hasMember("rate") &&
              behavior_list[i].hasMember("average_times") && behavior_list[i].hasMember("recognize_deltaX") &&
              behavior_list[i].hasMember("recognize_deltaY"))
          {
            defaultParam = false;
            no_tag.push_back(behavior_list[i]["no_tag"]);
            featureMatchingTimeout.push_back(behavior_list[i]["featureMatchingTimeout"]);
            stepControlTimeout.push_back(behavior_list[i]["stepControlTimeout"]);
            guiway_vel_line.push_back(behavior_list[i]["guiway_vel_line"]);
            guiway_vel_angle.push_back(behavior_list[i]["guiway_vel_angle"]);
            preparePosition.push_back(behavior_list[i]["preparePosition"]);
            delta_backDist.push_back(behavior_list[i]["delta_backDist"]);
            stop_recognizeDist.push_back(behavior_list[i]["stop_recognizeDist"]);
            obstacle_dist.push_back(behavior_list[i]["obstacle_dist"]);
            getout_vel.push_back(behavior_list[i]["getout_vel"]);
            scale.push_back(behavior_list[i]["scale"]);
            rate.push_back(behavior_list[i]["rate"]);
            average_times.push_back(behavior_list[i]["average_times"]);
            ROS_INFO("aic auto dock has got average_times");
            recognize_deltaX.push_back(behavior_list[i]["recognize_deltaX"]);
            recognize_deltaY.push_back(behavior_list[i]["recognize_deltaY"]);
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
        featureMatchingTimeout_ = featureMatchingTimeout.at(i);
        stepControlTimeout_ = stepControlTimeout.at(i);
        guiway_vel_line_ = guiway_vel_line.at(i);
        guiway_vel_angle_ = guiway_vel_angle.at(i);
        preparePosition_ = preparePosition.at(i);
        delta_backDist_ = delta_backDist.at(i);
        stop_recognizeDist_ = stop_recognizeDist.at(i);
        obstacle_dist_ = obstacle_dist.at(i);
        getout_vel_ = getout_vel.at(i);
        scale_ = scale.at(i);
        rate_ = rate.at(i);
        average_times_ = average_times.at(i);
        recognize_deltaX_ = recognize_deltaX.at(i);
        recognize_deltaY_ = recognize_deltaY.at(i);
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

    nh_local_.param< double >("featureMatchingTimeout", featureMatchingTimeout_, 5);
    nh_local_.param< double >("stepControlTimeout", stepControlTimeout_, 15);
    nh_local_.param< double >("guiway_vel_line", guiway_vel_line_, 0.0);
    nh_local_.param< double >("guiway_vel_angle", guiway_vel_angle_, 0.0);
    nh_local_.param< double >("preparePosition", preparePosition_, 1.0);
    nh_local_.param< double >("delta_backDist", delta_backDist_, 0.0);
    nh_local_.param< double >("stop_recognizeDist", stop_recognizeDist_, 0.0);
    nh_local_.param< double >("obstacle_dist", obstacle_dist_, 0.0);
    nh_local_.param< double >("getout_vel", getout_vel_, 0.2);
    nh_local_.param< double >("scale", scale_, 0.015);
    nh_local_.param< double >("rate", rate_, 15);
    nh_local_.param< double >("average_times", average_times_, 15);
    nh_local_.param< double >("recognize_deltaX", recognize_deltaX_, 0.0);
    nh_local_.param< double >("recognize_deltaY", recognize_deltaY_, 0.0);

    return false;
  }
  return true;
}

void autodock_interface::leave_run(const aic_msgs::AutoDock2GoalConstPtr& goal)
{
  ROS_INFO("Accept leave order");

  feedback_.err_msg = SYS_SECONDARY_LOCALIZATION_LEAVING_START;
  feedback_.status = aic_msgs::AutoDock2Feedback::EXECUTING;
  feedback_.feedback = aic_msgs::AutoDock2Feedback::LEGAL_GOAL;
  server_->publishFeedback(feedback_);
  status_code_.data = SYS_SECONDARY_LOCALIZATION_LEAVING_START;
  status_code_pub_.publish(status_code_);
  ros::Duration(0.1).sleep();

  if (Data_->module_data.currentStep == ModuleData::SuccessFinsh)
  {
    Data_->module_data.currentStep = ModuleData::SuccessFinsh;
    Data_->module_data.current_status = true;
    Data_->module_data.remaining_distance = 0.0;
    return;
  }
  else if (Data_->module_data.currentStep == ModuleData::RecognizeStep ||
           Data_->module_data.currentStep == ModuleData::Empty)
  {
    Data_->module_data.currentStep = ModuleData::SuccessFinsh;
    Data_->module_data.current_status = true;
    Data_->module_data.remaining_distance = 0.0;
    return;
  }
  else if (Data_->module_data.currentStep == ModuleData::DockingStep)
  {
    /***
    * Leaving off the shelf
    ***/
    double dist;
    if (Data_->dock_direction_ == docking_direction::backward)
      dist = fabs(fabs(preparePosition_) - half_length_ - Data_->module_data.remaining_distance);
    else
      dist = -fabs(fabs(preparePosition_) - half_length_ - Data_->module_data.remaining_distance);

    Data_->module_data.currentStep = ModuleData::GetOutStep;
    Data_->module_data.current_status = false;
    Data_->module_data.remaining_distance = 0.0;
    if (!leavingGetOut(dist))
    {
      Data_->module_data.remaining_distance = fabs(moveAvoid_remaining_distance_);
      return;
    }
    Data_->module_data.current_status = true;

    Data_->module_data.currentStep = ModuleData::SuccessFinsh;
    Data_->module_data.current_status = true;
    Data_->module_data.remaining_distance = 0.0;
    return;
  }
  else if (Data_->module_data.currentStep == ModuleData::GetOutStep)
  {
    /***
    * Leaving off the shelf
    ***/
    double dist;
    if (Data_->dock_direction_ == docking_direction::backward)
      dist = fabs(Data_->module_data.remaining_distance);
    else
      dist = -fabs(Data_->module_data.remaining_distance);

    Data_->module_data.currentStep = ModuleData::GetOutStep;
    Data_->module_data.current_status = false;
    Data_->module_data.remaining_distance = 0.0;
    if (!leavingGetOut(dist))
    {
      Data_->module_data.remaining_distance = fabs(moveAvoid_remaining_distance_);
      return;
    }
    Data_->module_data.current_status = true;

    Data_->module_data.currentStep = ModuleData::SuccessFinsh;
    Data_->module_data.current_status = true;
    Data_->module_data.remaining_distance = 0.0;
    return;
  }
}

bool autodock_interface::leavingGetOut(const double& dist)
{
  ROS_INFO("drive back in specified distance");
  ROS_INFO("preparePosition:%f, remaining_distance:%f", preparePosition_ - half_length_, dist);

  aic_msgs::OdomCtrolAvoidObstacleGoal line_control;
  if (!lineStepMotion(dist, fabs(getout_vel_), line_control, true)) // guiway_vel_line_
  {
    if (Data_->motion_status_ == failed)
    {
      status_code_.data = SYS_SECONDARY_LOCALIZATION_LEAVING_FAILED;
      status_code_pub_.publish(status_code_);
      results_.err_msg = SYS_SECONDARY_LOCALIZATION_LEAVING_FAILED;
      results_.result = aic_msgs::AutoDock2Result::FAILED;
      ROS_WARN("docking interface actionlib server: Motion fail");
    }
    else if (Data_->motion_status_ == timeout)
    {
      status_code_.data = SYS_SECONDARY_LOCALIZATION_LEAVING_FAILED;
      status_code_pub_.publish(status_code_);
      results_.err_msg = SYS_SECONDARY_LOCALIZATION_LEAVING_FAILED;
      results_.result = aic_msgs::AutoDock2Result::FAILED;
      ROS_WARN("docking interface actionlib server: Motion timeout");
    }
    return false;
  }
  return true;
}

void autodock_interface::robotInfoCallback(const aic_msgs::RobotInfo& msg)
{
  if (msg.laserNum == 0)
    ROS_ERROR_THROTTLE(2, "dock interface node accept the error robot information which lidar number is 0!");
  number_lidar_ = msg.laserNum;
}

void autodock_interface::setFootprintCallback(const geometry_msgs::Polygon& msg)
{
  vector< geometry_msgs::Point > points;
  double half_length = 0, half_width = 0;
  for (int i = 0; i < msg.points.size(); i++)
  {
    geometry_msgs::Point pt;
    pt.x = msg.points.at(i).x;
    pt.y = msg.points.at(i).y;
    points.push_back(pt);

    if (abs(msg.points.at(i).x) > half_length)
      half_length = abs(msg.points.at(i).x);
    if (abs(msg.points.at(i).y) > half_width)
      half_width = abs(msg.points.at(i).y);
  }
  half_length_ = half_length;
  half_width_ = half_width;

  /*** 避障区域 ***/
  points_polygon_.points.clear();
  points_polygon_padding_.points.clear();
  for (vector< geometry_msgs::Point >::iterator cit = points.begin(); cit != points.end(); cit++)
  {
    geometry_msgs::Point32 pt_padding;
    pt_padding.x = cit->x + (cit->x >= 0 ? 0.2 : -0.2);
    pt_padding.y = cit->y + (cit->y >= 0 ? 0.2 : -0.2);

    geometry_msgs::Point32 pt1;
    pt1.x = cit->x;
    pt1.y = cit->y;
    points_polygon_.points.push_back(pt1);
    points_polygon_padding_.points.push_back(pt_padding);
  }
}

void autodock_interface::LaserCallback(const sensor_msgs::LaserScan& msg)
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

    bool boolcarBodyPadding = pnpoly::polygon_contains(points_polygon_padding_, foot_pt_frame.getOrigin().getX(),
                                                       foot_pt_frame.getOrigin().getY());
    bool boolcarBody =
        pnpoly::polygon_contains(points_polygon_, foot_pt_frame.getOrigin().getX(), foot_pt_frame.getOrigin().getY());

    if (dist_laser > msg.range_min && dist_laser < msg.range_max && !boolcarBody && boolcarBodyPadding)
    {
      danger_mark_ = true;

      return;
    }
  }
  danger_mark_ = false;
}

void autodock_interface::pubMarkerCarStraightSquare()
{
  visualization_msgs::Marker marker_msg;

  marker_msg.ns = "docking avoid square extra_range";
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

  for (vector< geometry_msgs::Point32 >::iterator cit = points_polygon_padding_.points.begin();
       cit != points_polygon_padding_.points.end(); cit++)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->x;
    p_start.y = cit->y;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
  }

  p_start.x = points_polygon_padding_.points.begin()->x;
  p_start.y = points_polygon_padding_.points.begin()->y;
  marker_msg.points.push_back(p_start);

  marker_msg.lifetime = ros::Duration(0.3);

  marker_msg.header.frame_id = "base_footprint";
  marker_msg.header.stamp = ros::Time::now();
  marker_pub_.publish(marker_msg);
}

bool autodock_interface::goalAccept()
{
  if (server_->isPreemptRequested())
  {
    if (server_->isNewGoalAvailable())
    {
      aic_msgs::AutoDock2GoalConstPtr goal = server_->acceptNewGoal();

      if (goal->type == aic_msgs::AutoDock2Goal::PAUSE && actionlib_status_ == executing)
      {
        actionlib_last_status_ = actionlib_status_;
        actionlib_status_ = paused;
        ROS_INFO("dock interface actionlib server: paused!");
        feedback_.status = aic_msgs::AutoDock2Feedback::PAUSE;
        feedback_.feedback = aic_msgs::AutoDock2Feedback::LEGAL_GOAL;
        server_->publishFeedback(feedback_);
      }
      else if (goal->type == aic_msgs::AutoDock2Goal::RESUM && actionlib_status_ == paused)
      {
        actionlib_last_status_ = actionlib_status_;
        actionlib_status_ = executing;
        ROS_INFO("dock interface actionlib server: resum!");
        feedback_.status = aic_msgs::AutoDock2Feedback::EXECUTING;
        feedback_.feedback = aic_msgs::AutoDock2Feedback::LEGAL_GOAL;
        server_->publishFeedback(feedback_);
      }
      else
      {
        ROS_ERROR("dock interface actionlib server: Accept new goal error! type error! goal type:%d", goal->type);
        feedback_.status = (actionlib_status_ == executing) ? aic_msgs::AutoDock2Feedback::EXECUTING
                                                            : aic_msgs::AutoDock2Feedback::PAUSE;
        feedback_.feedback = aic_msgs::AutoDock2Feedback::ILLEGAL_GOAL;
        server_->publishFeedback(feedback_);
      }
    }
    else
      return false;
  }
  return true;
}

//7_add 
void autodock_interface::CB_simple_goal(const geometry_msgs::PoseStampedConstPtr& msg)
{
  tf::Vector3 goal_pos;
  tf::StampedTransform odom_map_frame;
  tf::Transform odom_goal_frame;
  tf::Transform map_goal_frame;
  tf::Quaternion q;
  q.setRPY(0,0,tf::getYaw(msg.get()->pose.orientation));
  listerner_.waitForTransform("odom", "map", ros::Time(0), ros::Duration(10.0));
  listerner_.lookupTransform("odom", "map", ros::Time(0), odom_map_frame);
  map_goal_frame.getOrigin().setValue(msg->pose.position.x,msg->pose.position.y,0);
  map_goal_frame.setRotation(q);
  nav_position_  = odom_map_frame*map_goal_frame;
  run_auto_dock_Thread_ = true;
  auto_dock_cond_.notify_one();
  ROS_WARN("aic auto dock accept move_base_simple/goal.");
  // aic_auto_dock::gui_way2Goal guiway_goal;
  // geometry_msgs::Pose goal_msg;
  // tf::poseTFToMsg(odom_goal_frame,goal_msg);
  // //aic_auto_dock::gui_way2Goal::BACK;
  // //aic_auto_dock::gui_way2Goal::STRAIGHT;
  // guiway_goal.type = aic_auto_dock::gui_way2Goal::STRAIGHT;
  // guiway_goal.pose = goal_msg;

  // guiway_goal.vel_line = 6;
  // guiway_goal.vel_angle = 1;
  // guiway_goal.back_dist = 0.1;//fabs(half_length_);
  // guiway_goal.obstacle_dist = 0.5;//fabs(half_length_);
  // guiway_goal.preparePosition = 2;
  // simple_goal_client_->waitForServer();
  // ROS_INFO("wait server succeed");

  // simple_goal_client_->sendGoal(guiway_goal);
}

void autodock_interface::AutoDockThread()
{
  ros::NodeHandle n;
  boost::unique_lock< boost::recursive_mutex > lock(auto_dock_mutex_);
  ros::Rate rate(25);
  double beginTime = ros::Time::now().toSec();
  double leftTime;

  while (n.ok())
  {
    while (!run_auto_dock_Thread_)
    {
      // if we should not be running the planner then suspend this thread
      ROS_INFO("auto dock thread, thread is suspending");
      auto_dock_cond_.wait(lock);

      beginTime = ros::Time::now().toSec();
    }
    lock.unlock();

    double dis = hypot(nav_position_.getOrigin().getX()- realTime_odom_.getOrigin().getX(),nav_position_.getOrigin().getY()- realTime_odom_.getOrigin().getY());
    if(dis<1.5)
    {
      client_auto_dock_->waitForServer();
      ROS_INFO("dock interface actionlib client: link auto dock successful!");
      aic_msgs::AutoDock2Goal goal;
      goal.tag_no = "0";
      goal.type   = 0;//ENTER;
      goal.board_shape = 2;//PARALLEL_SHAPE;
      goal.dock_direction  = 1;//FRONTWARD;
      client_auto_dock_->sendGoal(goal);
      run_auto_dock_Thread_ = false;
    }

    /***
     * timeout
     ***/
    if (ros::Time::now().toSec() - beginTime > AutoDockTimeout_ && AutoDockTimeout_ > 0)
    {
      ROS_WARN("auto dock thread, dock interface auto dock timeout");
      run_auto_dock_Thread_ = false;
    }
    lock.lock();
  }
}

void autodock_interface::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Vector3 vec(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  realTime_odom_.setOrigin(vec);
  realTime_odom_.setRotation(q);
}
