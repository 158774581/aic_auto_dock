#ifndef AIC_PLANNER_H
#define AIC_PLANNER_H

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
#include <teb_local_planner/teb_config.h>
#include <interactive_markers/interactive_marker_server.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ros/subscriber.h>

#include <alglib/interpolation.h>
#include <alglib/stdafx.h>

using namespace teb_local_planner;
using namespace std;
using namespace alglib;
//plan in odom
class teb_planner
{
  public:
    teb_planner(ros::NodeHandle& nh);
    teb_planner(ros::NodeHandle& nh,vector< geometry_msgs::Point > footprint);
    teb_planner(ros::NodeHandle& nh,geometry_msgs::Point& p1,geometry_msgs::Point& p2);
    ~teb_planner(){};
    bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses);
    bool setplan(PoseSE2& startpose,geometry_msgs::Twist& startVel,PoseSE2& endpose);
    PlannerInterfacePtr getplanner(){return planner_;};
    TebConfig& getTebConfig() {return config_;};
    void CB_publishCycle(const ros::TimerEvent& e);
    void CB_publishCycle();
    void setLineObstacle(float x0,float y0,float x1,float y1);
    void setPointObstacle(float x0,float y0);
    void setViaPoints(const nav_msgs::Path& via_points_msg);
    void interpolatePath(const tf::Transform& odom_tmp,const nav_msgs::Path& via_points_raw,nav_msgs::Path& path);
    void clearObstacle();
    bool isTrajFeasible();
   // RobotFootprintModelPtr setRobotFootprintPolygon(Point2dContainer polygon);
    RobotFootprintModelPtr setRobotFootprintPolygon( vector< geometry_msgs::Point > points);
    RobotFootprintModelPtr setRobotFootprintLine( geometry_msgs::Point& p1,geometry_msgs::Point& p2);
    // bool isTrajFeasible(PoseSE2 robot_pos, PoseSE2 goal_pos);
    // bool findGuiWayPath(vector<PoseSE2>& path);

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

#endif //AIC_PLANNER_H
