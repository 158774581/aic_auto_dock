#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "aic_auto_dock/aic_extraction.h"
#include "aic_auto_dock/math/aicpoly.h"
#include "aic_auto_dock/math/recognizeUtility.h"
#include "aic_auto_dock/math/status.h"
#include "aic_laser_feature_line_extraction/LineSegmentList.h"
#include "aic_laser_feature_line_extraction/line_extraction_ros.h"

using namespace std;

class ElevatorExtra : public AicExtraction
{
public:
  ElevatorExtra(ros::NodeHandle&, ros::NodeHandle&);
  ~ElevatorExtra() {}

  bool run(const geometry_msgs::Pose& old_middle_point, geometry_msgs::Pose& new_middle_point);
  void initLaserNav();
  bool setInterface(const AicExtractionInterface&);
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
                  vector< pair< int, int > >& temp_line_index);
  bool extraWallLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                      const vector< unsigned int >& line_index, vector< geometry_msgs::Pose >& vec_middle_point,
                      vector< pair< int, int > >& temp_line_index);
  bool extraRoomLines(const vector< recognize_extraction::extra_line >& input_extra_lines,
                      const vector< unsigned int >& line_index, vector< geometry_msgs::Pose >& vec_middle_point,
                      vector< pair< int, int > >& temp_line_index);

  void point32TFToMsg(const tf::Point&, geometry_msgs::Point32&);
  void loadParam();
  void initParam();
  void pubExtraRangeMarker();
  void pubElevatorRectangle(const geometry_msgs::Polygon& points, int id);

  ros::Publisher extra_range_marker_;
  ros::Publisher elevator_rectangle_marker_;
  tf::TransformListener listerner_;
  tf::TransformBroadcaster br_;

  double corner_min_angle_, corner_max_angle_, lines_min_gap_, lines_max_gap_, maxmun_length_, minimun_length_,
      lines_min_angle_, lines_max_angle_, elevator_depth_;
  double wall_min_length_, wall_max_length_;
  double max_split_lines_, room_max_gap_, room_min_gap_, room_max_line_length_, room_min_line_length_, room_delta_y_;
  double charging_port_, charging_port_delta_y_, charging_port_delta_angle_, delta_backDist_;
  tf::StampedTransform foot_laser_frame_, laser_laserNav_frame_;
  bool lidar_position_inverse_;
};
