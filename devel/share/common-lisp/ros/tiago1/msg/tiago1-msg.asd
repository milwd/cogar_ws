
(cl:in-package :asdf)

(defsystem "tiago1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Voice_rec" :depends-on ("_package_Voice_rec"))
    (:file "_package_Voice_rec" :depends-on ("_package"))
  ))