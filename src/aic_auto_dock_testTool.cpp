#include "aic_auto_dock/aic_auto_dock_testTool.h"
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <time.h>

//测试阵列计算函数
namespace aic
{
AutoDockTestTool::AutoDockTestTool(const ros::NodeHandle& nh)
    : nh_(nh), client_(new actionlib::SimpleActionClient< aic_msgs::StepControlAction >("step_control", true)),
      client_move_avoid_(
          new actionlib::SimpleActionClient< aic_msgs::OdomCtrolAvoidObstacleAction >("aic_move_avoid", true)),
      which_array_(0), x_max_range_(0.8), x_min_range_(0.4), x_array_num_(5), angle_max_range_(0.20 * M_PI),
      angle_min_range_(-0.20 * M_PI), angle_array_num_(20), successed_times_(0), successed_used_time_(0.0),
      failed_times_(0), test_vel_(0.7)
{
  nh_.param("x_max_range", x_max_range_, 0.8);
  nh_.param("x_min_range", x_min_range_, 0.3);
  nh_.param("x_array_num", x_array_num_, 5);
  nh_.param("angle_max_range", angle_max_range_, 0.2 * M_PI);
  nh_.param("angle_min_range", angle_min_range_, -0.20 * M_PI);
  nh_.param("angle_array_num", angle_array_num_, 20);
  nh_.param("test_vel", test_vel_, 0.7);
  nh_.param("dock_direction", dock_direction_, 0);
  nh_.param("waitingTime", waitingTime_, 5.0);
  nh_.param< std::string >("tag_no", tag_no_, "0");
  nh_.param("board_shape", board_shape_, 1);
  nh_.param("prepare_dist", prepare_dist_, 0.1);
  nh_.param("logic_mode", logic_mode_, 0);

  ActionClient_ = new actionlib::SimpleActionClient< aic_msgs::AutoDock2Action >("aic_auto_dock", true);

  client_->waitForServer();
  ROS_INFO("step control is ready!");

  ActionClient_->waitForServer();
  ROS_INFO("moveing to charge is ready!");

  client_move_avoid_->waitForServer();
  ROS_INFO("move avoid is ready!");

  timer_ = nh_.createTimer(ros::Duration(0.5), boost::bind(&AutoDockTestTool::timerCallback, this, _1));

  array_marker_pub_ = nh_.advertise< visualization_msgs::Marker >("/autodock/array_marker", 1);
  readyArrayComputeFunc();

  if (dock_direction_ != aic_msgs::AutoDock2Goal::BACKWARD && dock_direction_ != aic_msgs::AutoDock2Goal::FRONTWARD)
  {
    ROS_WARN("docking test tool using default direction!");
    dock_direction_ = aic_msgs::AutoDock2Goal::FRONTWARD;
  }

  if (board_shape_ == aic_msgs::AutoDock2Goal::VL_SHAPE)
    initLogFile("vlshape");
  else if (board_shape_ == aic_msgs::AutoDock2Goal::PARALLEL_SHAPE)
    initLogFile("parallelshape");
  else if (board_shape_ == aic_msgs::AutoDock2Goal::ELEVATOR_SHAPE)
    initLogFile("elevatorshape");
  else
  {
    ROS_WARN("docking test tool using default board shape!");
    initLogFile("vlshape");
  }

  /*** logic control ***/
  pub_batter_charge_ = nh_.advertise< aic_msgs::Charge >("/charge_cmd", 1);
  sub_batter_status_ = nh_.subscribe("/battery", 10, &AutoDockTestTool::batterStatusCallback, this);

  ROS_INFO("init success!");
}

AutoDockTestTool::~AutoDockTestTool()
{
  //平均时间为成功充电时所花的时间
  writeResultToLog(odom_array_.size(), successed_times_, failed_times_, successed_used_time_ / successed_times_);
  ROS_INFO("Autodock Test Finished! Please check log files in ~/.ros directory!");

  client_->cancelGoal();
  client_move_avoid_->cancelGoal();
  delete client_;
  delete log_writer_;
}

void AutoDockTestTool::readyArrayComputeFunc()
{
  double x_offset = (x_max_range_ - x_min_range_) / x_array_num_;
  double angle_offset = (angle_max_range_ - angle_min_range_) / angle_array_num_;
  for (size_t i = 0; i < angle_array_num_; i++)
  {
    std::pair< double, double > pos;
    pos.second = angle_min_range_ + i * angle_offset;
    for (size_t j = 0; j < x_array_num_; j++)
    {
      pos.first = x_min_range_ + j * x_offset;
      ready_array_.push_back(pos);
    }
  }
  odom_array_.clear();
  int id = 0;
  ros::Rate r(10);
  for (const auto& pair : ready_array_)
  {
    geometry_msgs::Pose2D odom_pose;
    odom_pose.x = 0.2 + pair.first * cos(pair.second);
    odom_pose.y = pair.first * sin(pair.second);
    odom_pose.theta = pair.second;
    odom_array_.push_back(odom_pose);
  }
}

//逻辑控制主函数
void AutoDockTestTool::timerCallback(const ros::TimerEvent& event)
{
  if (which_array_ >= odom_array_.size())
  {
    timer_.stop();
    //平均时间为成功充电时所花的时间
    writeResultToLog(odom_array_.size(), successed_times_, failed_times_, successed_used_time_ / successed_times_);
    ROS_INFO("Autodock Test Finished! Please check log files in ~/.ros directory!");
    nh_.shutdown();
    return;
  }
  double dist = hypot((odom_array_[which_array_].x - 0.2), odom_array_[which_array_].y);
  double angle = odom_array_[which_array_].theta;

  //直线行走离开充电桩
  ROS_INFO("Test: lineStepMotion");
  double prepare_dist = dock_direction_ == 0 ? prepare_dist_ : -prepare_dist_;
  if (!lineStepMotion(prepare_dist, test_vel_))
  {
    nh_.shutdown();
    ROS_ERROR("testTool fail, stop!!");
    return;
  }

  //旋转角度
  ROS_INFO("Test: rotationSetpMotion");
  if (!rotationStepMotion(angle, test_vel_))
  {
    nh_.shutdown();
    ROS_ERROR("testTool fail, stop!!");
    return;
  }

  //直线移动到阵列点
  ROS_INFO("Test: lineStepMotion");
  dist = dock_direction_ == 0 ? dist : -dist;
  if (!lineStepMotion(dist, test_vel_))
  {
    nh_.shutdown();
    ROS_ERROR("testTool fail, stop!!");
    return;
  }

  //旋转角度
  ROS_INFO("Test: rotationSetpMotion");
  if (!rotationStepMotion(-angle, test_vel_))
  {
    nh_.shutdown();
    ROS_ERROR("testTool fail, stop!!");
    return;
  }

  start_time_ = ros::Time::now();

  //发送进入指令
  goal_.tag_no = tag_no_;
  goal_.board_shape = board_shape_;
  goal_.type = aic_msgs::AutoDock2Goal::ENTER;
  goal_.dock_direction = dock_direction_;
  ActionClient_->sendGoal(goal_);
  ActionClient_->waitForResult();
  bool enter_is_ok = false;
  if (ActionClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    enter_is_ok = true;
  }
  else
  {
    ROS_ERROR("enter docking fail");
    enter_is_ok = false;
  }

  bool logic_is_ok = false;
  if (logicControl())
    logic_is_ok = true;
  else
  {
    ROS_ERROR("logic docking fail");
    logic_is_ok = false;
  }

  ros::Duration(waitingTime_).sleep();

  //发送退出指令
  goal_.tag_no = tag_no_;
  goal_.board_shape = board_shape_;
  goal_.type = aic_msgs::AutoDock2Goal::LEAVE;
  goal_.dock_direction = dock_direction_;
  ActionClient_->sendGoal(goal_);
  ActionClient_->waitForResult();
  bool leave_is_ok = false;
  if (ActionClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    leave_is_ok = true;
  }
  else
  {
    ROS_ERROR("leave docking fail");
    leave_is_ok = false;
  }

  end_time_ = ros::Time::now();
  double used_time = end_time_.toSec() - start_time_.toSec() - waitingTime_;

  bool is_ok = false;
  if (enter_is_ok && leave_is_ok && logic_is_ok)
  {
    ROS_INFO("success num%d", success_num);
    success_num++;
    is_ok = true;
  }
  else
    is_ok = false;

  ChargeStatus status;
  if (is_ok)
  {
    //统计成功次数
    ++successed_times_;
    successed_used_time_ += used_time;
    status = SUCCESSED;
    ROS_WARN("success No.%d used_time: %f", which_array_, used_time);
    writeRecordToLog(which_array_, odom_array_[which_array_].x - 0.2, odom_array_[which_array_].y, used_time, is_ok);
  }
  else
  {
    //统计失败次数
    ++failed_times_;
    status = FAILED;
    writeRecordToLog(which_array_, odom_array_[which_array_].x - 0.2, odom_array_[which_array_].y, used_time, is_ok);
    ROS_ERROR("Failed No.%d used_time: %f", which_array_, used_time);
  }
  ++which_array_;
}

bool AutoDockTestTool::logicControl()
{
  if (board_shape_ == aic_msgs::AutoDock2Goal::VL_SHAPE)
  {
    if (logic_mode_ == LogicMode::CHARGE)
    {
      ros::Duration(0.5).sleep();

      aic_msgs::Charge charge;
      charge.chargeMode = 1;
      pub_batter_charge_.publish(charge);

      double dist = dock_direction_ == 0 ? -0.03 : 0.03;
      if (!lineStepMotion(dist, 0.005, 1))
      {
        nh_.shutdown();
        ROS_ERROR("testTool fail, stop!!");
        return false;
      }

      for(int i = 0;i<=5;i++)
      {
        ros::Duration(1).sleep();

        if ((battery_.chargeStatus == 1) || (battery_.chargeStatus == 2))
          return true;
      }
      return false;
    }
  }
  else if (board_shape_ == aic_msgs::AutoDock2Goal::PARALLEL_SHAPE)
  {
    return true;
  }
  else if (board_shape_ == aic_msgs::AutoDock2Goal::ELEVATOR_SHAPE)
  {
    return true;
  }
  else
    return false;
}

bool AutoDockTestTool::lineStepMotion(double dist, double speed, int useMoveAvoid)
{
  aic_msgs::OdomCtrolAvoidObstacleGoal control;

  control.arg_type = 0;
  control.arg = dist;
  control.speed = speed;
  control.useMoveAvoid = useMoveAvoid;

  client_move_avoid_->sendGoal(control);
  client_move_avoid_->waitForResult();
  if (client_move_avoid_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("line success!");
    return true;
  }
  else
  {
    ROS_ERROR("line failed!");
    return false;
  }
}

bool AutoDockTestTool::rotationStepMotion(double angle, double speed, int useMoveAvoid)
{
  aic_msgs::OdomCtrolAvoidObstacleGoal control;

  control.arg_type = 1;
  control.arg = angle;
  control.speed = speed;
  control.useMoveAvoid = useMoveAvoid;

  client_move_avoid_->sendGoal(control);
  client_move_avoid_->waitForResult();
  if (client_move_avoid_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("rotate success!");
    return true;
  }
  else
  {
    ROS_ERROR("rotate failed!");
    return false;
  }
}

void AutoDockTestTool::writeRecordToLog(int num, double x, double y, double time, bool is_ok)
{
  *log_writer_ << std::setw(10) << num << std::setw(10) << x << std::setw(10) << y << std::setw(20) << time
               << std::setw(20) << is_ok << std::endl;
}

void AutoDockTestTool::writeResultToLog(int sum, int successed, int fail, double time)
{
  *log_writer_ << std::setw(10) << "total dock:" << sum << std::setw(20) << "successed:" << successed << std::setw(20)
               << "failed:" << fail << std::setw(20) << "average time:" << time << std::endl;
}

void AutoDockTestTool::initLogFile(std::string name)
{
  aicrobot::FileOperation file_operate;
  std::string dir = "/home/aicrobo/.aiclog/testTool";
  if (!file_operate.makeDir(dir))
  {
    ROS_ERROR("%s is not exiting, failed to make it", dir.c_str());
    return;
  }

  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  struct tm* parts = std::localtime(&now_c);
  std::string year = boost::lexical_cast< std::string >(parts->tm_year + 1900);
  std::string month = boost::lexical_cast< std::string >(parts->tm_mon + 1);
  std::string day = boost::lexical_cast< std::string >(parts->tm_mday);
  std::string hour = boost::lexical_cast< std::string >(parts->tm_hour);
  std::string min = boost::lexical_cast< std::string >(parts->tm_min);
  std::string sec = boost::lexical_cast< std::string >(parts->tm_sec);
  std::string file_name = name + "-" + year + "-" + month + "-" + day + "-" + hour + "-" + min + "-" + sec;
  log_writer_ = new std::ofstream("/home/aicrobo/.aiclog/testTool/" + file_name + ".log", std::ios::app);
  *log_writer_ << std::setw(10) << "no." << std::setw(10) << "x" << std::setw(10) << "y" << std::setw(20) << "time"
               << std::setw(20) << "is_ok" << std::endl;
}

void AutoDockTestTool::sendMarker(uint32_t id, double x, double y, ChargeStatus status)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  if (status == UNREACHED)
  {
    marker.color.b = 1.0;
  }
  else if (status == SUCCESSED)
  {
    marker.color.g = 1.0;
  }
  else if (status == FAILED)
  {
    marker.color.r = 1.0;
  }
  marker.color.a = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.id = id;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "autodock";
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.05;
  marker.pose.orientation.w = 1.0;
  array_marker_pub_.publish(marker);
}

void AutoDockTestTool::batterStatusCallback(const aic_msgs::Battery::ConstPtr& msg)
{
  battery_.chargeStatus = msg->chargeStatus;
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aic_auto_dock_testTool");
  ros::NodeHandle nh("~");
  aic::AutoDockTestTool adt(nh);
  ros::MultiThreadedSpinner spin;
  spin.spin();
  return 0;
}
