// Generated by gencpp from file aic_auto_dock/gui_way2Action.msg
// DO NOT EDIT!


#ifndef AIC_AUTO_DOCK_MESSAGE_GUI_WAY2ACTION_H
#define AIC_AUTO_DOCK_MESSAGE_GUI_WAY2ACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <aic_auto_dock/gui_way2ActionGoal.h>
#include <aic_auto_dock/gui_way2ActionResult.h>
#include <aic_auto_dock/gui_way2ActionFeedback.h>

namespace aic_auto_dock
{
template <class ContainerAllocator>
struct gui_way2Action_
{
  typedef gui_way2Action_<ContainerAllocator> Type;

  gui_way2Action_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  gui_way2Action_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::aic_auto_dock::gui_way2ActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::aic_auto_dock::gui_way2ActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::aic_auto_dock::gui_way2ActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> const> ConstPtr;

}; // struct gui_way2Action_

typedef ::aic_auto_dock::gui_way2Action_<std::allocator<void> > gui_way2Action;

typedef boost::shared_ptr< ::aic_auto_dock::gui_way2Action > gui_way2ActionPtr;
typedef boost::shared_ptr< ::aic_auto_dock::gui_way2Action const> gui_way2ActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aic_auto_dock::gui_way2Action_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace aic_auto_dock

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'aic_auto_dock': ['/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/aic_auto_dock/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8d6be3d579a5c9e2f446499313a96936";
  }

  static const char* value(const ::aic_auto_dock::gui_way2Action_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8d6be3d579a5c9e2ULL;
  static const uint64_t static_value2 = 0xf446499313a96936ULL;
};

template<class ContainerAllocator>
struct DataType< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aic_auto_dock/gui_way2Action";
  }

  static const char* value(const ::aic_auto_dock::gui_way2Action_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
gui_way2ActionGoal action_goal\n\
gui_way2ActionResult action_result\n\
gui_way2ActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: aic_auto_dock/gui_way2ActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
gui_way2Goal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: aic_auto_dock/gui_way2Goal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
string              tag_no\n\
int32               type\n\
geometry_msgs/Pose  pose\n\
float32             vel_line\n\
float32             vel_angle\n\
float32             back_dist\n\
float32             obstacle_dist\n\
float32             preparePosition\n\
float32             scale            #角速度响应比例，取值范围：0~0.1\n\
\n\
int32               BACK = 0\n\
int32               STRAIGHT = 1\n\
int32               PAUSE = 2\n\
int32               RESUM = 3\n\
int32               INITPORT = 4\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: aic_auto_dock/gui_way2ActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
gui_way2Result result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: aic_auto_dock/gui_way2Result\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
uint64              err_msg          #状态码\n\
\n\
int32               result\n\
int32               CANCLE = 1\n\
int32               SUCCESS = 2\n\
int32               FAILED = 3\n\
int32               INITFAILED = 4\n\
\n\
float32             remaining_distance\n\
\n\
================================================================================\n\
MSG: aic_auto_dock/gui_way2ActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
gui_way2Feedback feedback\n\
\n\
================================================================================\n\
MSG: aic_auto_dock/gui_way2Feedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
uint64              err_msg          #状态码\n\
\n\
int32               status\n\
int32               EXECUTING = 1\n\
int32               PAUSE = 2\n\
\n\
int32               feedback\n\
int32               OBSTACLE_AVOIDING = 10\n\
int32               AVOID_SUCCESS = 11\n\
int32               ILLEGAL_GOAL = 12\n\
int32               STEP_PROCESS = 13\n\
\n\
float32             remaining_distance\n\
int32               step_process\n\
int32               PREPARE_NAV_STEP = 0\n\
int32               PREPARE_STEP = 1\n\
int32               PORT_STEP = 2\n\
\n\
";
  }

  static const char* value(const ::aic_auto_dock::gui_way2Action_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gui_way2Action_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aic_auto_dock::gui_way2Action_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aic_auto_dock::gui_way2Action_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::aic_auto_dock::gui_way2ActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::aic_auto_dock::gui_way2ActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::aic_auto_dock::gui_way2ActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AIC_AUTO_DOCK_MESSAGE_GUI_WAY2ACTION_H
