
(cl:in-package :asdf)

(defsystem "tiago1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :nav_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MovementControlAction" :depends-on ("_package_MovementControlAction"))
    (:file "_package_MovementControlAction" :depends-on ("_package"))
    (:file "MovementControlActionFeedback" :depends-on ("_package_MovementControlActionFeedback"))
    (:file "_package_MovementControlActionFeedback" :depends-on ("_package"))
    (:file "MovementControlActionGoal" :depends-on ("_package_MovementControlActionGoal"))
    (:file "_package_MovementControlActionGoal" :depends-on ("_package"))
    (:file "MovementControlActionResult" :depends-on ("_package_MovementControlActionResult"))
    (:file "_package_MovementControlActionResult" :depends-on ("_package"))
    (:file "MovementControlFeedback" :depends-on ("_package_MovementControlFeedback"))
    (:file "_package_MovementControlFeedback" :depends-on ("_package"))
    (:file "MovementControlGoal" :depends-on ("_package_MovementControlGoal"))
    (:file "_package_MovementControlGoal" :depends-on ("_package"))
    (:file "MovementControlResult" :depends-on ("_package_MovementControlResult"))
    (:file "_package_MovementControlResult" :depends-on ("_package"))
  ))