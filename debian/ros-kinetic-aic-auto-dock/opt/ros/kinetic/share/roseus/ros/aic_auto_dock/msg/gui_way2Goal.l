;; Auto-generated. Do not edit!


(when (boundp 'aic_auto_dock::gui_way2Goal)
  (if (not (find-package "AIC_AUTO_DOCK"))
    (make-package "AIC_AUTO_DOCK"))
  (shadow 'gui_way2Goal (find-package "AIC_AUTO_DOCK")))
(unless (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL")
  (make-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))

(in-package "ROS")
;;//! \htmlinclude gui_way2Goal.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*BACK*" (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(shadow '*BACK* (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(defconstant aic_auto_dock::gui_way2Goal::*BACK* 0)
(intern "*STRAIGHT*" (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(shadow '*STRAIGHT* (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(defconstant aic_auto_dock::gui_way2Goal::*STRAIGHT* 1)
(intern "*PAUSE*" (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(shadow '*PAUSE* (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(defconstant aic_auto_dock::gui_way2Goal::*PAUSE* 2)
(intern "*RESUM*" (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(shadow '*RESUM* (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(defconstant aic_auto_dock::gui_way2Goal::*RESUM* 3)
(intern "*INITPORT*" (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(shadow '*INITPORT* (find-package "AIC_AUTO_DOCK::GUI_WAY2GOAL"))
(defconstant aic_auto_dock::gui_way2Goal::*INITPORT* 4)
(defclass aic_auto_dock::gui_way2Goal
  :super ros::object
  :slots (_tag_no _type _pose _vel_line _vel_angle _back_dist _obstacle_dist _preparePosition _scale ))

(defmethod aic_auto_dock::gui_way2Goal
  (:init
   (&key
    ((:tag_no __tag_no) "")
    ((:type __type) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:vel_line __vel_line) 0.0)
    ((:vel_angle __vel_angle) 0.0)
    ((:back_dist __back_dist) 0.0)
    ((:obstacle_dist __obstacle_dist) 0.0)
    ((:preparePosition __preparePosition) 0.0)
    ((:scale __scale) 0.0)
    )
   (send-super :init)
   (setq _tag_no (string __tag_no))
   (setq _type (round __type))
   (setq _pose __pose)
   (setq _vel_line (float __vel_line))
   (setq _vel_angle (float __vel_angle))
   (setq _back_dist (float __back_dist))
   (setq _obstacle_dist (float __obstacle_dist))
   (setq _preparePosition (float __preparePosition))
   (setq _scale (float __scale))
   self)
  (:tag_no
   (&optional __tag_no)
   (if __tag_no (setq _tag_no __tag_no)) _tag_no)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:vel_line
   (&optional __vel_line)
   (if __vel_line (setq _vel_line __vel_line)) _vel_line)
  (:vel_angle
   (&optional __vel_angle)
   (if __vel_angle (setq _vel_angle __vel_angle)) _vel_angle)
  (:back_dist
   (&optional __back_dist)
   (if __back_dist (setq _back_dist __back_dist)) _back_dist)
  (:obstacle_dist
   (&optional __obstacle_dist)
   (if __obstacle_dist (setq _obstacle_dist __obstacle_dist)) _obstacle_dist)
  (:preparePosition
   (&optional __preparePosition)
   (if __preparePosition (setq _preparePosition __preparePosition)) _preparePosition)
  (:scale
   (&optional __scale)
   (if __scale (setq _scale __scale)) _scale)
  (:serialization-length
   ()
   (+
    ;; string _tag_no
    4 (length _tag_no)
    ;; int32 _type
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; float32 _vel_line
    4
    ;; float32 _vel_angle
    4
    ;; float32 _back_dist
    4
    ;; float32 _obstacle_dist
    4
    ;; float32 _preparePosition
    4
    ;; float32 _scale
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _tag_no
       (write-long (length _tag_no) s) (princ _tag_no s)
     ;; int32 _type
       (write-long _type s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; float32 _vel_line
       (sys::poke _vel_line (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vel_angle
       (sys::poke _vel_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _back_dist
       (sys::poke _back_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _obstacle_dist
       (sys::poke _obstacle_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _preparePosition
       (sys::poke _preparePosition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _scale
       (sys::poke _scale (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _tag_no
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tag_no (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float32 _vel_line
     (setq _vel_line (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vel_angle
     (setq _vel_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _back_dist
     (setq _back_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _obstacle_dist
     (setq _obstacle_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _preparePosition
     (setq _preparePosition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _scale
     (setq _scale (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get aic_auto_dock::gui_way2Goal :md5sum-) "0df86edf95e2372a391a8bf1928e52fe")
(setf (get aic_auto_dock::gui_way2Goal :datatype-) "aic_auto_dock/gui_way2Goal")
(setf (get aic_auto_dock::gui_way2Goal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
string              tag_no
int32               type
geometry_msgs/Pose  pose
float32             vel_line
float32             vel_angle
float32             back_dist
float32             obstacle_dist
float32             preparePosition
float32             scale            #角速度响应比例，取值范围：0~0.1

int32               BACK = 0
int32               STRAIGHT = 1
int32               PAUSE = 2
int32               RESUM = 3
int32               INITPORT = 4

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :aic_auto_dock/gui_way2Goal "0df86edf95e2372a391a8bf1928e52fe")


