
(cl:in-package :asdf)

(defsystem "aic_auto_dock-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "gui_way2Action" :depends-on ("_package_gui_way2Action"))
    (:file "_package_gui_way2Action" :depends-on ("_package"))
    (:file "gui_way2ActionFeedback" :depends-on ("_package_gui_way2ActionFeedback"))
    (:file "_package_gui_way2ActionFeedback" :depends-on ("_package"))
    (:file "gui_way2ActionGoal" :depends-on ("_package_gui_way2ActionGoal"))
    (:file "_package_gui_way2ActionGoal" :depends-on ("_package"))
    (:file "gui_way2ActionResult" :depends-on ("_package_gui_way2ActionResult"))
    (:file "_package_gui_way2ActionResult" :depends-on ("_package"))
    (:file "gui_way2Feedback" :depends-on ("_package_gui_way2Feedback"))
    (:file "_package_gui_way2Feedback" :depends-on ("_package"))
    (:file "gui_way2Goal" :depends-on ("_package_gui_way2Goal"))
    (:file "_package_gui_way2Goal" :depends-on ("_package"))
    (:file "gui_way2Result" :depends-on ("_package_gui_way2Result"))
    (:file "_package_gui_way2Result" :depends-on ("_package"))
  ))