#ifndef AIC_PLANNER_H
#define AIC_PLANNER_H

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
#include <teb_local_planner/teb_config.h>
#include <interactive_markers/interactive_marker_server.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ros/subscriber.h>

using namespace teb_local_planner;
using namespace std;

//plan in odom
class teb_planner
{
  public:
    teb_planner(ros::NodeHandle& nh);
    ~teb_planner(){};
    bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses);
    bool setplan(PoseSE2& startpose,geometry_msgs::Twist& startVel,PoseSE2& endpose);
    PlannerInterfacePtr getplanner(){return planner_;};
    TebConfig& getTebConfig() {return config_;};
    void CB_publishCycle(const ros::TimerEvent& e);
    void CB_publishCycle();
    void setLineObstacle(float x0,float y0,float x1,float y1);
    void setViaPoints(const nav_msgs::Path& via_points_msg);
    void clearObstacle();
    bool isTrajFeasible(double vx, double vy, double omega);
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

#endif //AIC_PLANNER_H