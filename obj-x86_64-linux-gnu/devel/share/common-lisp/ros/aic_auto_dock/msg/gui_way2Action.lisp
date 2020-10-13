; Auto-generated. Do not edit!


(cl:in-package aic_auto_dock-msg)


;//! \htmlinclude gui_way2Action.msg.html

(cl:defclass <gui_way2Action> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type aic_auto_dock-msg:gui_way2ActionGoal
    :initform (cl:make-instance 'aic_auto_dock-msg:gui_way2ActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type aic_auto_dock-msg:gui_way2ActionResult
    :initform (cl:make-instance 'aic_auto_dock-msg:gui_way2ActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type aic_auto_dock-msg:gui_way2ActionFeedback
    :initform (cl:make-instance 'aic_auto_dock-msg:gui_way2ActionFeedback)))
)

(cl:defclass gui_way2Action (<gui_way2Action>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gui_way2Action>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gui_way2Action)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aic_auto_dock-msg:<gui_way2Action> is deprecated: use aic_auto_dock-msg:gui_way2Action instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <gui_way2Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:action_goal-val is deprecated.  Use aic_auto_dock-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <gui_way2Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:action_result-val is deprecated.  Use aic_auto_dock-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <gui_way2Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:action_feedback-val is deprecated.  Use aic_auto_dock-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gui_way2Action>) ostream)
  "Serializes a message object of type '<gui_way2Action>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gui_way2Action>) istream)
  "Deserializes a message object of type '<gui_way2Action>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gui_way2Action>)))
  "Returns string type for a message object of type '<gui_way2Action>"
  "aic_auto_dock/gui_way2Action")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gui_way2Action)))
  "Returns string type for a message object of type 'gui_way2Action"
  "aic_auto_dock/gui_way2Action")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gui_way2Action>)))
  "Returns md5sum for a message object of type '<gui_way2Action>"
  "8d6be3d579a5c9e2f446499313a96936")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gui_way2Action)))
  "Returns md5sum for a message object of type 'gui_way2Action"
  "8d6be3d579a5c9e2f446499313a96936")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gui_way2Action>)))
  "Returns full string definition for message of type '<gui_way2Action>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%gui_way2ActionGoal action_goal~%gui_way2ActionResult action_result~%gui_way2ActionFeedback action_feedback~%~%================================================================================~%MSG: aic_auto_dock/gui_way2ActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%gui_way2Goal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: aic_auto_dock/gui_way2Goal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%string              tag_no~%int32               type~%geometry_msgs/Pose  pose~%float32             vel_line~%float32             vel_angle~%float32             back_dist~%float32             obstacle_dist~%float32             preparePosition~%float32             scale            #角速度响应比例，取值范围：0~~0.1~%~%int32               BACK = 0~%int32               STRAIGHT = 1~%int32               PAUSE = 2~%int32               RESUM = 3~%int32               INITPORT = 4~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: aic_auto_dock/gui_way2ActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%gui_way2Result result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: aic_auto_dock/gui_way2Result~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint64              err_msg          #状态码~%~%int32               result~%int32               CANCLE = 1~%int32               SUCCESS = 2~%int32               FAILED = 3~%int32               INITFAILED = 4~%~%float32             remaining_distance~%~%================================================================================~%MSG: aic_auto_dock/gui_way2ActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%gui_way2Feedback feedback~%~%================================================================================~%MSG: aic_auto_dock/gui_way2Feedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint64              err_msg          #状态码~%~%int32               status~%int32               EXECUTING = 1~%int32               PAUSE = 2~%~%int32               feedback~%int32               OBSTACLE_AVOIDING = 10~%int32               AVOID_SUCCESS = 11~%int32               ILLEGAL_GOAL = 12~%int32               STEP_PROCESS = 13~%~%float32             remaining_distance~%int32               step_process~%int32               PREPARE_NAV_STEP = 0~%int32               PREPARE_STEP = 1~%int32               PORT_STEP = 2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gui_way2Action)))
  "Returns full string definition for message of type 'gui_way2Action"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%gui_way2ActionGoal action_goal~%gui_way2ActionResult action_result~%gui_way2ActionFeedback action_feedback~%~%================================================================================~%MSG: aic_auto_dock/gui_way2ActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%gui_way2Goal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: aic_auto_dock/gui_way2Goal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%string              tag_no~%int32               type~%geometry_msgs/Pose  pose~%float32             vel_line~%float32             vel_angle~%float32             back_dist~%float32             obstacle_dist~%float32             preparePosition~%float32             scale            #角速度响应比例，取值范围：0~~0.1~%~%int32               BACK = 0~%int32               STRAIGHT = 1~%int32               PAUSE = 2~%int32               RESUM = 3~%int32               INITPORT = 4~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: aic_auto_dock/gui_way2ActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%gui_way2Result result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: aic_auto_dock/gui_way2Result~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint64              err_msg          #状态码~%~%int32               result~%int32               CANCLE = 1~%int32               SUCCESS = 2~%int32               FAILED = 3~%int32               INITFAILED = 4~%~%float32             remaining_distance~%~%================================================================================~%MSG: aic_auto_dock/gui_way2ActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%gui_way2Feedback feedback~%~%================================================================================~%MSG: aic_auto_dock/gui_way2Feedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint64              err_msg          #状态码~%~%int32               status~%int32               EXECUTING = 1~%int32               PAUSE = 2~%~%int32               feedback~%int32               OBSTACLE_AVOIDING = 10~%int32               AVOID_SUCCESS = 11~%int32               ILLEGAL_GOAL = 12~%int32               STEP_PROCESS = 13~%~%float32             remaining_distance~%int32               step_process~%int32               PREPARE_NAV_STEP = 0~%int32               PREPARE_STEP = 1~%int32               PORT_STEP = 2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gui_way2Action>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gui_way2Action>))
  "Converts a ROS message object to a list"
  (cl:list 'gui_way2Action
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))