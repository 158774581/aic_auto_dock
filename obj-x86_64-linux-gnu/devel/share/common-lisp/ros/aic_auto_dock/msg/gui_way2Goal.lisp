; Auto-generated. Do not edit!


(cl:in-package aic_auto_dock-msg)


;//! \htmlinclude gui_way2Goal.msg.html

(cl:defclass <gui_way2Goal> (roslisp-msg-protocol:ros-message)
  ((tag_no
    :reader tag_no
    :initarg :tag_no
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (vel_line
    :reader vel_line
    :initarg :vel_line
    :type cl:float
    :initform 0.0)
   (vel_angle
    :reader vel_angle
    :initarg :vel_angle
    :type cl:float
    :initform 0.0)
   (back_dist
    :reader back_dist
    :initarg :back_dist
    :type cl:float
    :initform 0.0)
   (obstacle_dist
    :reader obstacle_dist
    :initarg :obstacle_dist
    :type cl:float
    :initform 0.0)
   (preparePosition
    :reader preparePosition
    :initarg :preparePosition
    :type cl:float
    :initform 0.0)
   (scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0))
)

(cl:defclass gui_way2Goal (<gui_way2Goal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gui_way2Goal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gui_way2Goal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aic_auto_dock-msg:<gui_way2Goal> is deprecated: use aic_auto_dock-msg:gui_way2Goal instead.")))

(cl:ensure-generic-function 'tag_no-val :lambda-list '(m))
(cl:defmethod tag_no-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:tag_no-val is deprecated.  Use aic_auto_dock-msg:tag_no instead.")
  (tag_no m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:type-val is deprecated.  Use aic_auto_dock-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:pose-val is deprecated.  Use aic_auto_dock-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'vel_line-val :lambda-list '(m))
(cl:defmethod vel_line-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:vel_line-val is deprecated.  Use aic_auto_dock-msg:vel_line instead.")
  (vel_line m))

(cl:ensure-generic-function 'vel_angle-val :lambda-list '(m))
(cl:defmethod vel_angle-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:vel_angle-val is deprecated.  Use aic_auto_dock-msg:vel_angle instead.")
  (vel_angle m))

(cl:ensure-generic-function 'back_dist-val :lambda-list '(m))
(cl:defmethod back_dist-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:back_dist-val is deprecated.  Use aic_auto_dock-msg:back_dist instead.")
  (back_dist m))

(cl:ensure-generic-function 'obstacle_dist-val :lambda-list '(m))
(cl:defmethod obstacle_dist-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:obstacle_dist-val is deprecated.  Use aic_auto_dock-msg:obstacle_dist instead.")
  (obstacle_dist m))

(cl:ensure-generic-function 'preparePosition-val :lambda-list '(m))
(cl:defmethod preparePosition-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:preparePosition-val is deprecated.  Use aic_auto_dock-msg:preparePosition instead.")
  (preparePosition m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <gui_way2Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:scale-val is deprecated.  Use aic_auto_dock-msg:scale instead.")
  (scale m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<gui_way2Goal>)))
    "Constants for message type '<gui_way2Goal>"
  '((:BACK . 0)
    (:STRAIGHT . 1)
    (:PAUSE . 2)
    (:RESUM . 3)
    (:INITPORT . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'gui_way2Goal)))
    "Constants for message type 'gui_way2Goal"
  '((:BACK . 0)
    (:STRAIGHT . 1)
    (:PAUSE . 2)
    (:RESUM . 3)
    (:INITPORT . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gui_way2Goal>) ostream)
  "Serializes a message object of type '<gui_way2Goal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tag_no))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tag_no))
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_line))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'back_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'obstacle_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'preparePosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gui_way2Goal>) istream)
  "Deserializes a message object of type '<gui_way2Goal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_no) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tag_no) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_line) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'back_dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'preparePosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gui_way2Goal>)))
  "Returns string type for a message object of type '<gui_way2Goal>"
  "aic_auto_dock/gui_way2Goal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gui_way2Goal)))
  "Returns string type for a message object of type 'gui_way2Goal"
  "aic_auto_dock/gui_way2Goal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gui_way2Goal>)))
  "Returns md5sum for a message object of type '<gui_way2Goal>"
  "0df86edf95e2372a391a8bf1928e52fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gui_way2Goal)))
  "Returns md5sum for a message object of type 'gui_way2Goal"
  "0df86edf95e2372a391a8bf1928e52fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gui_way2Goal>)))
  "Returns full string definition for message of type '<gui_way2Goal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%string              tag_no~%int32               type~%geometry_msgs/Pose  pose~%float32             vel_line~%float32             vel_angle~%float32             back_dist~%float32             obstacle_dist~%float32             preparePosition~%float32             scale            #角速度响应比例，取值范围：0~~0.1~%~%int32               BACK = 0~%int32               STRAIGHT = 1~%int32               PAUSE = 2~%int32               RESUM = 3~%int32               INITPORT = 4~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gui_way2Goal)))
  "Returns full string definition for message of type 'gui_way2Goal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%string              tag_no~%int32               type~%geometry_msgs/Pose  pose~%float32             vel_line~%float32             vel_angle~%float32             back_dist~%float32             obstacle_dist~%float32             preparePosition~%float32             scale            #角速度响应比例，取值范围：0~~0.1~%~%int32               BACK = 0~%int32               STRAIGHT = 1~%int32               PAUSE = 2~%int32               RESUM = 3~%int32               INITPORT = 4~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gui_way2Goal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tag_no))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gui_way2Goal>))
  "Converts a ROS message object to a list"
  (cl:list 'gui_way2Goal
    (cl:cons ':tag_no (tag_no msg))
    (cl:cons ':type (type msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':vel_line (vel_line msg))
    (cl:cons ':vel_angle (vel_angle msg))
    (cl:cons ':back_dist (back_dist msg))
    (cl:cons ':obstacle_dist (obstacle_dist msg))
    (cl:cons ':preparePosition (preparePosition msg))
    (cl:cons ':scale (scale msg))
))
