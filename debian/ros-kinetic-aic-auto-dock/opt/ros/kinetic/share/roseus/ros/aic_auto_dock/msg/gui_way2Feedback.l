;; Auto-generated. Do not edit!


(when (boundp 'aic_auto_dock::gui_way2Feedback)
  (if (not (find-package "AIC_AUTO_DOCK"))
    (make-package "AIC_AUTO_DOCK"))
  (shadow 'gui_way2Feedback (find-package "AIC_AUTO_DOCK")))
(unless (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK")
  (make-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude gui_way2Feedback.msg.html


(intern "*EXECUTING*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*EXECUTING* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*EXECUTING* 1)
(intern "*PAUSE*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*PAUSE* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*PAUSE* 2)
(intern "*OBSTACLE_AVOIDING*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*OBSTACLE_AVOIDING* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*OBSTACLE_AVOIDING* 10)
(intern "*AVOID_SUCCESS*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*AVOID_SUCCESS* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*AVOID_SUCCESS* 11)
(intern "*ILLEGAL_GOAL*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*ILLEGAL_GOAL* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*ILLEGAL_GOAL* 12)
(intern "*STEP_PROCESS*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*STEP_PROCESS* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*STEP_PROCESS* 13)
(intern "*PREPARE_NAV_STEP*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*PREPARE_NAV_STEP* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*PREPARE_NAV_STEP* 0)
(intern "*PREPARE_STEP*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*PREPARE_STEP* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*PREPARE_STEP* 1)
(intern "*PORT_STEP*" (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(shadow '*PORT_STEP* (find-package "AIC_AUTO_DOCK::GUI_WAY2FEEDBACK"))
(defconstant aic_auto_dock::gui_way2Feedback::*PORT_STEP* 2)
(defclass aic_auto_dock::gui_way2Feedback
  :super ros::object
  :slots (_err_msg _status _feedback _remaining_distance _step_process ))

(defmethod aic_auto_dock::gui_way2Feedback
  (:init
   (&key
    ((:err_msg __err_msg) 0)
    ((:status __status) 0)
    ((:feedback __feedback) 0)
    ((:remaining_distance __remaining_distance) 0.0)
    ((:step_process __step_process) 0)
    )
   (send-super :init)
   (setq _err_msg (round __err_msg))
   (setq _status (round __status))
   (setq _feedback (round __feedback))
   (setq _remaining_distance (float __remaining_distance))
   (setq _step_process (round __step_process))
   self)
  (:err_msg
   (&optional __err_msg)
   (if __err_msg (setq _err_msg __err_msg)) _err_msg)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:feedback
   (&optional __feedback)
   (if __feedback (setq _feedback __feedback)) _feedback)
  (:remaining_distance
   (&optional __remaining_distance)
   (if __remaining_distance (setq _remaining_distance __remaining_distance)) _remaining_distance)
  (:step_process
   (&optional __step_process)
   (if __step_process (setq _step_process __step_process)) _step_process)
  (:serialization-length
   ()
   (+
    ;; uint64 _err_msg
    8
    ;; int32 _status
    4
    ;; int32 _feedback
    4
    ;; float32 _remaining_distance
    4
    ;; int32 _step_process
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _err_msg
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _err_msg (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _err_msg) (= (length (_err_msg . bv)) 2)) ;; bignum
              (write-long (ash (elt (_err_msg . bv) 0) 0) s)
              (write-long (ash (elt (_err_msg . bv) 1) -1) s))
             ((and (class _err_msg) (= (length (_err_msg . bv)) 1)) ;; big1
              (write-long (elt (_err_msg . bv) 0) s)
              (write-long (if (>= _err_msg 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _err_msg s)(write-long (if (>= _err_msg 0) 0 #xffffffff) s)))
     ;; int32 _status
       (write-long _status s)
     ;; int32 _feedback
       (write-long _feedback s)
     ;; float32 _remaining_distance
       (sys::poke _remaining_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _step_process
       (write-long _step_process s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _err_msg
     
#+(or :alpha :irix6 :x86_64)
      (setf _err_msg (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _err_msg (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int32 _status
     (setq _status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _feedback
     (setq _feedback (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _remaining_distance
     (setq _remaining_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _step_process
     (setq _step_process (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get aic_auto_dock::gui_way2Feedback :md5sum-) "92d80fef0082d27f007cd916743cbfcb")
(setf (get aic_auto_dock::gui_way2Feedback :datatype-) "aic_auto_dock/gui_way2Feedback")
(setf (get aic_auto_dock::gui_way2Feedback :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
uint64              err_msg          #状态码

int32               status
int32               EXECUTING = 1
int32               PAUSE = 2

int32               feedback
int32               OBSTACLE_AVOIDING = 10
int32               AVOID_SUCCESS = 11
int32               ILLEGAL_GOAL = 12
int32               STEP_PROCESS = 13

float32             remaining_distance
int32               step_process
int32               PREPARE_NAV_STEP = 0
int32               PREPARE_STEP = 1
int32               PORT_STEP = 2


")



(provide :aic_auto_dock/gui_way2Feedback "92d80fef0082d27f007cd916743cbfcb")


