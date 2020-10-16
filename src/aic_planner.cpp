/*
  author :7
  date   :2020.10.9
*/
#include <aic_auto_dock/aic_planner.h>

//------------ teb planner ----------------
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
    if(isTrajFeasible() == true)
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
void teb_planner::setPointObstacle(float x0,float y0)
{
  obst_vector_.push_back( boost::make_shared<PointObstacle>(x0, y0));
}

// set obstacle
void teb_planner::clearObstacle()
{
  obst_vector_.clear();
}

bool teb_planner::isTrajFeasible()
{
  PoseSE2 predict_pose(startpose_);
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
      for (const ObstaclePtr& obst : obst_vector_)
      {        
        double obs_dis = robot_model_->calculateDistance(predict_pose,obst.get());
        if(obs_dis < config_.obstacles.min_obstacle_dist)
        {
          ROS_INFO_ONCE("robot is too close to obstacles,distance %f %f!!!",config_.obstacles.min_obstacle_dist,obs_dis);
          return false;
        }
      }
    }
  }
  else
  {
    //后续补充
  }

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

void teb_planner::interpolatePath(const tf::Transform& odom_tmp,\
                                  const nav_msgs::Path& via_points_raw,nav_msgs::Path& path)
{
  tf::Quaternion q;
  q.setRPY(0,0,0);
  tf::Transform tmp_via_frame(q);
  tf::Transform odom_via_frame(q);
  geometry_msgs::PoseStamped via_p;
  real_2d_array xy;
  pspline2interpolant spline;
  unsigned int n = via_points_raw.poses.size();
  double points_raw[n*2];
  double cfg_dt = 0.01;
  double cfg_path_length = getTebConfig().trajectory.max_global_plan_lookahead_dist;
  bool cfg_overwrite = getTebConfig().trajectory.global_plan_overwrite_orientation;
  cfg_overwrite = true;
  double x,y;
  for(int i=0;i<n;i++)
  {
    points_raw[2*i] = via_points_raw.poses[i].pose.position.x;
    points_raw[2*i+1] = via_points_raw.poses[i].pose.position.y;
  }
  xy.setcontent(n,2,points_raw);
  pspline2build(xy, n, 0,0,spline);
  
  double path_length = 0;
  for (double t=1;t>=0;t=t-cfg_dt)
  {
    pspline2calc(spline, t,x,y);
    tmp_via_frame.setOrigin(tf::Vector3(x,y,0.0));
    path_length = pspline2arclength(spline,t,1);

    odom_via_frame  = odom_tmp*tmp_via_frame; 
    via_p.pose.position.x = odom_via_frame.getOrigin().getX();
    via_p.pose.position.y = odom_via_frame.getOrigin().getY();
    tf::quaternionTFToMsg(odom_via_frame.getRotation(),via_p.pose.orientation);
    path.poses.push_back(via_p);
    if(path_length > cfg_path_length)
    {
      break;
    }
  }
  setViaPoints(path);
}


