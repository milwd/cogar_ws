
(cl:in-package :asdf)

(defsystem "tiago1-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :tiago1-msg
)
  :components ((:file "_package")
    (:file "robotstatedecision" :depends-on ("_package_robotstatedecision"))
    (:file "_package_robotstatedecision" :depends-on ("_package"))
    (:file "send_order" :depends-on ("_package_send_order"))
    (:file "_package_send_order" :depends-on ("_package"))
  ))