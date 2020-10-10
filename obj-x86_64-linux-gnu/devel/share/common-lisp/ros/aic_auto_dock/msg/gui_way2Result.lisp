; Auto-generated. Do not edit!


(cl:in-package aic_auto_dock-msg)


;//! \htmlinclude gui_way2Result.msg.html

(cl:defclass <gui_way2Result> (roslisp-msg-protocol:ros-message)
  ((err_msg
    :reader err_msg
    :initarg :err_msg
    :type cl:integer
    :initform 0)
   (result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0)
   (remaining_distance
    :reader remaining_distance
    :initarg :remaining_distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass gui_way2Result (<gui_way2Result>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gui_way2Result>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gui_way2Result)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aic_auto_dock-msg:<gui_way2Result> is deprecated: use aic_auto_dock-msg:gui_way2Result instead.")))

(cl:ensure-generic-function 'err_msg-val :lambda-list '(m))
(cl:defmethod err_msg-val ((m <gui_way2Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:err_msg-val is deprecated.  Use aic_auto_dock-msg:err_msg instead.")
  (err_msg m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <gui_way2Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:result-val is deprecated.  Use aic_auto_dock-msg:result instead.")
  (result m))

(cl:ensure-generic-function 'remaining_distance-val :lambda-list '(m))
(cl:defmethod remaining_distance-val ((m <gui_way2Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aic_auto_dock-msg:remaining_distance-val is deprecated.  Use aic_auto_dock-msg:remaining_distance instead.")
  (remaining_distance m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<gui_way2Result>)))
    "Constants for message type '<gui_way2Result>"
  '((:CANCLE . 1)
    (:SUCCESS . 2)
    (:FAILED . 3)
    (:INITFAILED . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'gui_way2Result)))
    "Constants for message type 'gui_way2Result"
  '((:CANCLE . 1)
    (:SUCCESS . 2)
    (:FAILED . 3)
    (:INITFAILED . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gui_way2Result>) ostream)
  "Serializes a message object of type '<gui_way2Result>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'err_msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'err_msg)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gui_way2Result>) istream)
  "Deserializes a message object of type '<gui_way2Result>"
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
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'remaining_distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gui_way2Result>)))
  "Returns string type for a message object of type '<gui_way2Result>"
  "aic_auto_dock/gui_way2Result")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gui_way2Result)))
  "Returns string type for a message object of type 'gui_way2Result"
  "aic_auto_dock/gui_way2Result")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gui_way2Result>)))
  "Returns md5sum for a message object of type '<gui_way2Result>"
  "e81f05ebaf1c3c2557540a3cd981a074")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gui_way2Result)))
  "Returns md5sum for a message object of type 'gui_way2Result"
  "e81f05ebaf1c3c2557540a3cd981a074")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gui_way2Result>)))
  "Returns full string definition for message of type '<gui_way2Result>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint64              err_msg          #状态码~%~%int32               result~%int32               CANCLE = 1~%int32               SUCCESS = 2~%int32               FAILED = 3~%int32               INITFAILED = 4~%~%float32             remaining_distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gui_way2Result)))
  "Returns full string definition for message of type 'gui_way2Result"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint64              err_msg          #状态码~%~%int32               result~%int32               CANCLE = 1~%int32               SUCCESS = 2~%int32               FAILED = 3~%int32               INITFAILED = 4~%~%float32             remaining_distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gui_way2Result>))
  (cl:+ 0
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gui_way2Result>))
  "Converts a ROS message object to a list"
  (cl:list 'gui_way2Result
    (cl:cons ':err_msg (err_msg msg))
    (cl:cons ':result (result msg))
    (cl:cons ':remaining_distance (remaining_distance msg))
))
