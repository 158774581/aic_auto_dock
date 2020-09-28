#ifndef GUI_WAY_H
#define GUI_WAY_H

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <aic_auto_dock/gui_way2Action.h>
#include <aic_auto_dock/math/footprint.h>
#include <aic_auto_dock/math/pnpoly.hpp>
#include <aic_auto_dock/math/recognizeUtility.h>
#include <aic_msgs/Error.h>
#include <aic_msgs/RobotInfo.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <aic_auto_dock/math/status.h>

//reb 
#include <teb_local_planner/teb_local_planner_ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ros/subscriber.h>

typedef actionlib::SimpleActionClient< aic_auto_dock::gui_way2Action > Client_gui_way;
//end (teb)
typedef actionlib::SimpleActionServer< aic_auto_dock::gui_way2Action > Server;
class teb_planner;

using namespace std;
using namespace recognize_extraction;
using namespace teb_local_planner;

struct detector
{
  double dist;
  double weight;
  int num;
};

struct AvoidType
{
  enum Avoid_type
  {
    ROUND,
    RECTANGLE
  };
};

class gui_way
{
public:
  gui_way(ros::NodeHandle&, ros::NodeHandle&);
  ~gui_way() {}

  void start(const aic_auto_dock::gui_way2GoalConstPtr&);

private:
  bool initDetector(double& percent, tf::Transform frame, double detector_y_coordinate);

  void initFrame(void);
  void workingFrame(void);

  void initStepProcess(const double& allowable_dist);

  void odomCallback(const nav_msgs::OdometryConstPtr&);
  void setFootprintCallback(const geometry_msgs::Polygon& msg);
  void LaserCallback(const sensor_msgs::LaserScan& msg);
  void CB_simple_goal(const geometry_msgs::PoseStampedConstPtr& msg);

  bool goalAccept();

  void pubMarkerCarStraightSquare();
  void pubMarkerCarTurnSquare();
  void pubVirtualPath();

  bool loadParamFromYaml();
  void cleanProcess(void);

  bool findGlobalPath();
  Server* as_;

  ros::NodeHandle local_nh_, nh_;
  ros::Subscriber setFootprint_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher twist_pub_;
  ros::Subscriber simple_goal_sub_;
  tf::TransformListener listerner_;
  ros::Subscriber sub_robot_exception_;
  ros::Publisher marker_pub_CarRadius;
  ros::Publisher marker_pub_, virtual_path_pub_;
  tf::TransformBroadcaster br_;

  /*** 接口 ***/
  double back_dist_ = 0.0, obstacle_dist_ = 0.0;
  docking_direction direction_;
  double preparePosition_;

  /*** launch ***/
  double delta_detector_spacing_;
  double dangerRange_x_, dangerRange_y_;
  double yawThrehold_, radiusThrehold_, detector_y_coordinate_;
  double scale_;
  double aW_acc_, aW_dcc_;
  double prepare_lineVel_, prepare_angleVel_, prepare_scale_, angleVel_turn_;

  /*** global variable ***/
  tf::Transform realTime_odom_, port_foot_frame, prepare_foot_frame, prepareNav_foot_frame, odom_port_frame_,
      target_foot_frame;
  geometry_msgs::Twist realTime_twist_;
  double prepareNavAngle_, preparePositionNav;
  StepProcess process_;

  double dangerRange_xMax_, dangerRange_xMin_, dangerRange_yMax_, dangerRange_yMin_;
  bool danger_mark_ = false;
  tf::StampedTransform foot_laser_frame_;
  double half_length_ = 0.0, half_width_ = 0.0;
  bool accept_robotInfo_ = false, accept_laserScan = false;
  AvoidType::Avoid_type avoidType_; // 0:circle  1:square

  actionlibStatus actionlib_status_;
  status motion_status_;

  vector< geometry_msgs::Point > points_;
  geometry_msgs::Polygon points_polygon_, points_straight_polygon_padding_, points_turn_polygon_padding_;

  double vel_line_, vel_angle_;
  //teb
  vector<PoseSE2> global_path_;
  double goal_inflation_x_,goal_inflation_y_;

  teb_planner* planner_;
  Client_gui_way* simple_goal_client_;
};

//plan in odom
class teb_planner
{
  public:
    teb_planner(ros::NodeHandle& nh);
    ~teb_planner(){};
    bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses);
    bool setplan(PoseSE2& startpose,geometry_msgs::Twist& startVel,PoseSE2& endpose);
    PlannerInterfacePtr getplanner(){return planner_;};
    void CB_publishCycle(const ros::TimerEvent& e);
    void CB_publishCycle();
    void setLineObstacle(float x0,float y0,float x1,float y1);
    void setViaPoints(const nav_msgs::Path& via_points_msg);
    void clearObstacle();
    bool isTrajFeasible(double vx, double vy, double omega, double look_ahead_time);
    bool isTrajFeasible(PoseSE2 robot_pos, PoseSE2 goal_pos);
    bool findGuiWayPath(vector<PoseSE2>& path);

  private:
    ros::NodeHandle nh_;
    PlannerInterfacePtr planner_;
    std::vector<ObstaclePtr> obst_vector_;
    ViaPointContainer via_points_;
    TebConfig config_;
    boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
    TebVisualizationPtr visual_;
    RobotFootprintModelPtr robot_model_;
    
    PoseSE2 startpose_; //!< Store current robot pose
    PoseSE2 endpose_; //!< Store current robot goal
    geometry_msgs::Twist startvel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
};

#endif // GUI_WAY_H
