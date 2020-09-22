#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "aic_laser_feature_line_extraction/LineSegmentList.h"
#include "aic_laser_feature_line_extraction/line_extraction_ros.h"
#include "aic_auto_dock/math/aicpoly.h"
#include "aic_auto_dock/math/recognizeUtility.h"
#include <Eigen/Dense>
#include <aic_msgs/OdomCtrolAvoidObstacleAction.h>
#include "aic_auto_dock/aic_extraction.h"

using namespace std;

class VShapeExtra : public AicExtraction
{
public:
  VShapeExtra(ros::NodeHandle&, ros::NodeHandle&);
  ~VShapeExtra(){}

  bool run(const geometry_msgs::Pose& old_middle_point, geometry_msgs::Pose& new_middle_point);

  bool setInterface(const AicExtractionInterface& );
  bool setParam(std::string tag, std::string name);

private:
  bool infoPreprocess(const aic_laser_feature_line_extraction::LineSegmentList& lines,
                      vector< recognize_extraction::extra_line >& input_extra_lines,
                      vector< unsigned int >& line_index);

  /**
   * @brief extraLines
   * @param input_extra_lines: input parameter, parent frame is base_footprint
   * @param line_index: input parameter, index
   * @param vec_middle_point: output parameter, parent frame is odom
   * @param temp_line_index: output parameter, index
   * @return
   */
  bool extraLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                  const vector< unsigned int >& line_index, vector< geometry_msgs::Pose >& vec_middle_point,
                  vector< tuple< size_t, size_t, size_t > >& temp_line_index);

  bool legalVShape(const vector< geometry_msgs::Pose >& input_extra_port,
                                vector< geometry_msgs::Pose >& vec_middle_point,
                                vector< tuple< size_t, size_t, size_t > >& temp_line_index);

  void point32TFToMsg(const tf::Point&, geometry_msgs::Point32&);
  void loadParam();
  void initParam();
  void pubExtraRangeMarker();
  void pubVShapeRectangle(const geometry_msgs::Polygon& points, int id);
  void populateMarkerMsg(const aic_laser_feature_line_extraction::LineSegmentList& lines, visualization_msgs::Marker& marker_msg, int id);


  ros::Publisher extra_range_marker_, vshape_rectangle_marker_, marker_publisher_;
  tf::TransformListener listerner_;
  tf::TransformBroadcaster br_;

  double max_lines_1_2_angle_, min_lines_1_2_angle_, max_lines_2_3_angle_, min_lines_2_3_angle_, max_lines_1_3_angle_,
      min_lines_1_3_angle_, max_lines1_, min_lines1_, max_lines2_, min_lines2_, max_lines3_, min_lines3_;
  double charging_port_, charging_port_delta_y_, charging_port_delta_angle_, delta_backDist_;

  tf::StampedTransform foot_laser_frame_, laser_laserNav_frame_;
  bool lidar_position_inverse_;

  double angle_vel_, last_angle_vel_ = 0.0;
  double dtScanTime_;
  bool accept_odom_vel_;
};
