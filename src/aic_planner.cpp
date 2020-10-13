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

// bool teb_planner::isTrajFeasible(PoseSE2 robot_pos, PoseSE2 goal_pos)
// {
//     for (ObstContainer::const_iterator obst = obst_vector_.begin(); obst != obst_vector_.end(); ++obst)
//     {
//       boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);   
//       if (!pobst)
//         continue;
//       double obs_dis = pobst->getMinimumDistance(robot_pos.position(),goal_pos.position());
//       //double obs_dis = robot_model_->calculateDistance(predict_pose,pobst.get());
//       if(obs_dis < 0.1)
//       {
//         return false;
//       }
//     }
//   // }

//   return true;
// }

void teb_planner::setViaPoints(const nav_msgs::Path& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg.poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}
//end(teb)
