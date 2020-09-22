#ifndef AUTODOCK_TEST_H
#define AUTODOCK_TEST_H

#include "aic_msgs/OdomCtrolAvoidObstacleAction.h"
#include "aic_msgs/AutoDock2Action.h"
#include <actionlib/client/simple_action_client.h>
#include <aic_msgs/Battery.h>
#include <aic_msgs/StepControlAction.h>
#include <fstream>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <aic_msgs/Charge.h>
#include <file_operation.h>

namespace aic
{
class AutoDockTestTool
{
public:
  AutoDockTestTool(const ros::NodeHandle& nh);
  ~AutoDockTestTool();
  int success_num;

private:
  enum ChargeStatus
  {
    UNREACHED = 0,
    SUCCESSED = 1,
    FAILED = 2
  };

  enum LogicMode
  {
    CHARGE = 0
  };

  bool logicControl();
  void readyArrayComputeFunc();
  void timerCallback(const ros::TimerEvent& event);
  bool lineStepMotion(double dist, double speed, int useMoveAvoid = 0);
  bool rotationStepMotion(double angle, double speed, int useMoveAvoid = 0);
  void writeRecordToLog(int num, double x, double y, double time, bool is_ok);
  void writeResultToLog(int sum, int successed, int fail, double time);
  void initLogFile(std::string name);
  void sendMarker(uint32_t id, double x, double y, ChargeStatus status);

  void batterStatusCallback(const aic_msgs::Battery::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Publisher go_dock_pub_;
  ros::Publisher array_marker_pub_;
  ros::Subscriber charge_status_sub_;
  ros::Subscriber odom_sub_;
  ros::Timer timer_;

  /*** logic control ***/
  ros::Publisher pub_batter_charge_;
  ros::Subscriber sub_batter_status_;
  aic_msgs::Battery battery_;

  actionlib::SimpleActionClient< aic_msgs::AutoDock2Action >* ActionClient_;
  geometry_msgs::Pose2D odom_;
  std::vector< std::pair< double, double > > ready_array_;
  std::vector< geometry_msgs::Pose2D > odom_array_;
  double x_min_range_;
  double x_max_range_;
  double angle_min_range_;
  double angle_max_range_;
  double test_vel_;
  int dock_direction_;
  double waitingTime_;
  double prepare_dist_;
  int logic_mode_;
  std::string tag_no_;
  int board_shape_;
  int x_array_num_;
  int angle_array_num_;
  uint32_t which_array_;
  ros::Time start_time_;
  ros::Time end_time_;

  double successed_used_time_;
  uint32_t successed_times_;
  uint32_t failed_times_;
  actionlib::SimpleActionClient< aic_msgs::StepControlAction >* client_;
  actionlib::SimpleActionClient< aic_msgs::OdomCtrolAvoidObstacleAction >* client_move_avoid_;
  std::ofstream* log_writer_;
  aic_msgs::AutoDock2Goal goal_;
};
}

#endif // AUTODOCK_TEST_H
