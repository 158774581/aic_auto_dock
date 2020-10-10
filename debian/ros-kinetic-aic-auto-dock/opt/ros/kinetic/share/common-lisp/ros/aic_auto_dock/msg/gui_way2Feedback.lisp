; Auto-generated. Do not edit!


(cl:in-package aic_auto_dock-msg)


;//! \htmlinclude gui_way2Feedback.msg.html

(cl:defclass <gui_way2Feedback> (roslisp-msg-protocol:ros-message)
  ((err_msg
    :reader err_msg
    :initarg :err_msg
    :type cl:integer
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0)
   (feedback
    :reader feedback
    :initarg :feedback
    :type cl:integer
    :initform 0)
   (remaining_distance
    :reader remaining_distance
    :initarg :remaining_distance
    :type cl:float
    :initform 0.0)
   (step_process
    :reader step_process
    :initarg :step_process
    :type cl:integer
    :initform 0))
)

(cl:defclass gui_way2Feedback (<gui_way2Feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gui_way2Feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gui_way2Feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aic_auto_dock-msg:<gui_way2Feedback> is deprecated: use aic_auto_dock-msg:gui_way2Feedback instead.")))

(cl:ensure-generic-function 'err_msg-val :lambda-list '(m))
(cl:defmethod err_msg-val ((m <gui_way2Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:err_msg-val is deprecated.  Use aic_auto_dock-msg:err_msg instead.")
  (err_msg m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <gui_way2Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:status-val is deprecated.  Use aic_auto_dock-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <gui_way2Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:feedback-val is deprecated.  Use aic_auto_dock-msg:feedback instead.")
  (feedback m))

(cl:ensure-generic-function 'remaining_distance-val :lambda-list '(m))
(cl:defmethod remaining_distance-val ((m <gui_way2Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:remaining_distance-val is deprecated.  Use aic_auto_dock-msg:remaining_distance instead.")
  (remaining_distance m))

(cl:ensure-generic-function 'step_process-val :lambda-list '(m))
(cl:defmethod step_process-val ((m <gui_way2Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:step_process-val is deprecated.  Use aic_auto_dock-msg:step_process instead.")
  (step_process m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<gui_way2Feedback>)))
    "Constants for message type '<gui_way2Feedback>"
  '((:EXECUTING . 1)
    (:PAUSE . 2)
    (:OBSTACLE_AVOIDING . 10)
    (:AVOID_SUCCESS . 11)
    (:ILLEGAL_GOAL . 12)
    (:STEP_PROCESS . 13)
    (:PREPARE_NAV_STEP . 0)
    (:PREPARE_STEP . 1)
    (:PORT_STEP . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'gui_way2Feedback)))
    "Constants for message type 'gui_way2Feedback"
  '((:EXECUTING . 1)
    (:PAUSE . 2)
    (:OBSTACLE_AVOIDING . 10)
    (:AVOID_SUCCESS . 11)
    (:ILLEGAL_GOAL . 12)
    (:STEP_PROCESS . 13)
    (:PREPARE_NAV_STEP . 0)
    (:PREPARE_STEP . 1)
    (:PORT_STEP . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gui_way2Feedback>) ostream)
  "Serializes a message object of type '<gui_way2Feedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'err_msg)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'feedback)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'remaining_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'step_process)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gui_way2Feedback>) istream)
  "Deserializes a message object of type '<gui_way2Feedback>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'err_msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'err_msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'err_msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'err_msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'err_msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'err_msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'err_msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'err_msg)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'feedback) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'remaining_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'step_process) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gui_way2Feedback>)))
  "Returns string type for a message object of type '<gui_way2Feedback>"
  "aic_auto_dock/gui_way2Feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gui_way2Feedback)))
  "Returns string type for a message object of type 'gui_way2Feedback"
  "aic_auto_dock/gui_way2Feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gui_way2Feedback>)))
  "Returns md5sum for a message object of type '<gui_way2Feedback>"
  "92d80fef0082d27f007cd916743cbfcb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gui_way2Feedback)))
  "Returns md5sum for a message object of type 'gui_way2Feedback"
  "92d80fef0082d27f007cd916743cbfcb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gui_way2Feedback>)))
  "Returns full string definition for message of type '<gui_way2Feedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint64              err_msg          #状态码~%~%int32               status~%int32               EXECUTING = 1~%int32               PAUSE = 2~%~%int32               feedback~%int32               OBSTACLE_AVOIDING = 10~%int32               AVOID_SUCCESS = 11~%int32               ILLEGAL_GOAL = 12~%int32               STEP_PROCESS = 13~%~%float32             remaining_distance~%int32               step_process~%int32               PREPARE_NAV_STEP = 0~%int32               PREPARE_STEP = 1~%int32               PORT_STEP = 2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gui_way2Feedback)))
  "Returns full string definition for message of type 'gui_way2Feedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint64              err_msg          #状态码~%~%int32               status~%int32               EXECUTING = 1~%int32               PAUSE = 2~%~%int32               feedback~%int32               OBSTACLE_AVOIDING = 10~%int32               AVOID_SUCCESS = 11~%int32               ILLEGAL_GOAL = 12~%int32               STEP_PROCESS = 13~%~%float32             remaining_distance~%int32               step_process~%int32               PREPARE_NAV_STEP = 0~%int32               PREPARE_STEP = 1~%int32               PORT_STEP = 2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gui_way2Feedback>))
  (cl:+ 0
     8
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gui_way2Feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'gui_way2Feedback
    (cl:cons ':err_msg (err_msg msg))
    (cl:cons ':status (status msg))
    (cl:cons ':feedback (feedback msg))
    (cl:cons ':remaining_distance (remaining_distance msg))
    (cl:cons ':step_process (step_process msg))
))
