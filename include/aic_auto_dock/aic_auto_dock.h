#ifndef AUTODOCK_INTERFACE_H
#define AUTODOCK_INTERFACE_H

#include "actionlib/server/simple_action_server.h"
#include "aic_auto_dock/gui_way2Action.h"
#include "aic_auto_dock/math/footprint.h"
#include "aic_auto_dock/math/pnpoly.hpp"
#include "aic_auto_dock/math/recognizeUtility.h"
#include "aic_auto_dock/math/status.h"
#include "aic_auto_dock/parallel_line_extra.h"
#include "aic_auto_dock/v_shape_extra.h"
#include "aic_auto_dock/elevator_extra.h"
#include "aic_msgs/AutoDock2Action.h"
#include "aic_msgs/RobotInfo.h"
#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include <aic_auto_dock/math/log.h>
#include <aic_msgs/ModuleStatus.h>

using namespace std;

typedef actionlib::SimpleActionServer< aic_msgs::AutoDock2Action > Server;
typedef actionlib::SimpleActionClient< aic_auto_dock::gui_way2Action > Client_gui_way;
typedef actionlib::SimpleActionClient< aic_msgs::OdomCtrolAvoidObstacleAction > Client_move_avoid;

class autodock_data
{
public:
  autodock_data() {}
  ~autodock_data() {}

  /*** global param ***/
  docking_direction dock_direction_;
  ModuleData module_data;
  status transform_status_ = empty, motion_status_ = empty;
};

class autodock_interface
{
public:
  autodock_interface(ros::NodeHandle&, ros::NodeHandle&);
  ~autodock_interface();

private:
  bool initparam();
  void run(const aic_msgs::AutoDock2GoalConstPtr& goal);
  bool initgoal(const aic_msgs::AutoDock2GoalConstPtr& goal);

  void Start(const aic_msgs::AutoDock2GoalConstPtr&);

  bool recognizeLogic(geometry_msgs::Pose& precise_pose, bool roughMark);
  bool recognize(const docking_direction& moving, geometry_msgs::Pose& precise_pose);
  bool recognizeMovingStrategy(const docking_direction& moving, const double& featureMatchingTimeout,
                               double& beginTime);
  bool recognizeThread(const double& featureMatchingTimeout, status& thread_status, double& beginTime,
                       double& leftTime);

  bool guiwayMotionWithVshapeCalibration(aic_auto_dock::gui_way2Goal& goal);
  bool guiwayMotionWithParallelCalibration(aic_auto_dock::gui_way2Goal& goal);
  bool guiwayMotionWithElevatorCalibration(aic_auto_dock::gui_way2Goal& goal);

  void initPolygon(const geometry_msgs::Pose& temp_pose, AicExtractionPolygon& polygon, double delta_y);
  void GuiWayThread();
  void getGuiWayCallback(const actionlib::SimpleClientGoalState& state,
                         const aic_auto_dock::gui_way2ResultConstPtr& result);
  void gui_way_feedbackCB(const aic_auto_dock::gui_way2FeedbackConstPtr& msg);
  void cleanProcess();

  bool lineStepMotion(double dist, double speed, aic_msgs::OdomCtrolAvoidObstacleGoal control, bool BoolAvoid);
  bool rotationStepMotion(double angle, double speed, aic_msgs::OdomCtrolAvoidObstacleGoal control, bool BoolAvoid);
  void MoveAvoidThread();
  void move_avoid_feedbackCB(const aic_msgs::OdomCtrolAvoidObstacleFeedbackConstPtr& msg);

  void leave_run(const aic_msgs::AutoDock2GoalConstPtr& goal);
  bool leavingGetOut(const double& dist);

  bool loadParamFromYaml(std::string tag, std::string name);
  void robotInfoCallback(const aic_msgs::RobotInfo& msg);
  void setFootprintCallback(const geometry_msgs::Polygon& msg);
  void LaserCallback(const sensor_msgs::LaserScan& msg);
  bool goalAccept();
  void pubMarkerCarStraightSquare();

  /**** ros param ****/
  ros::NodeHandle nh_, nh_local_;
  ros::Publisher status_code_pub_, twist_pub_, marker_pub_;
  ros::Subscriber robot_info_sub_, setFootprint_sub_, laser_sub_;
  Server* server_;
  Client_gui_way* client_gui_way_;
  tf::TransformListener listerner_;
  tf::TransformBroadcaster br_;
  Client_move_avoid* client_move_avoid_;

  aic_msgs::AutoDock2Feedback feedback_;
  aic_msgs::AutoDock2Result results_;
  std_msgs::UInt64 status_code_;

  VShapeExtra* v_shape_extra_;
  ParallelLineExtra* parallel_line_extra_;
  ElevatorExtra* elevator_extra_;
  AicExtraction* extra_pt_;
  line_extraction::LineExtractionROS* line_extractor_;

  autodock_data *Data_, *VShapeData_, *ParallelLineData_, *ElevatorData_;

  bool (autodock_interface::*pfun)(aic_auto_dock::gui_way2Goal&);

  /*** launch ***/
  string scan_frame_;
  tf::StampedTransform foot_laser_frame_;

  /*** global param ***/
  actionlibStatus actionlib_status_, actionlib_last_status_, guiway_status_;
  geometry_msgs::Polygon points_polygon_, points_polygon_padding_;
  StepProcess process_;
  double guiWay_remaining_distance_, guiWayFB_remaining_distance_, moveAvoid_remaining_distance_;

  double half_length_ = 0.0, half_width_ = 0.0, carRadius_ = 0.0;
  int number_lidar_ = 1;
  bool danger_mark_ = false;

  // thread
  boost::thread *move_avoid_thread_, *gui_way_thread_;
  boost::recursive_mutex move_avoid_mutex_, gui_way_mutex_;
  bool run_move_avoid_Thread_ = false, run_gui_way_Thread_ = false;
  boost::condition_variable_any move_avoid_cond_, gui_way_cond_;
  status move_avoid_thread_status_ = empty, gui_way_thread_status_ = empty;

  /*** yaml ***/
  double featureMatchingTimeout_, stepControlTimeout_;
  double guiway_vel_line_, guiway_vel_angle_, guiway_vel_last_line_, guiway_vel_last_angle_;
  double preparePosition_, delta_backDist_, stop_recognizeDist_, obstacle_dist_, getout_vel_, scale_, rate_, average_times_;
  double recognize_deltaX_, recognize_deltaY_;
};



#endif // AUTODOCK_INTERFACE_H
