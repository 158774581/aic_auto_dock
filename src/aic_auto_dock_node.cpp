#include "aic_auto_dock/aic_auto_dock.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aic_auto_dock");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  autodock_interface auto_dock(nh, nh_local);

  ros::spin();
  //  ros::MultiThreadedSpinner spin;
  //  spin.spin();
  return 0;
}
