;; Auto-generated. Do not edit!


(when (boundp 'tiago1::robotstatedecision)
  (if (not (find-package "TIAGO1"))
    (make-package "TIAGO1"))
  (shadow 'robotstatedecision (find-package "TIAGO1")))
(unless (find-package "TIAGO1::ROBOTSTATEDECISION")
  (make-package "TIAGO1::ROBOTSTATEDECISION"))
(unless (find-package "TIAGO1::ROBOTSTATEDECISIONREQUEST")
  (make-package "TIAGO1::ROBOTSTATEDECISIONREQUEST"))
(unless (find-package "TIAGO1::ROBOTSTATEDECISIONRESPONSE")
  (make-package "TIAGO1::ROBOTSTATEDECISIONRESPONSE"))

(in-package "ROS")





(defclass tiago1::robotstatedecisionRequest
  :super ros::object
  :slots (_state_input ))

(defmethod tiago1::robotstatedecisionRequest
  (:init
   (&key
    ((:state_input __state_input) "")
    )
   (send-super :init)
   (setq _state_input (string __state_input))
   self)
  (:state_input
   (&optional __state_input)
   (if __state_input (setq _state_input __state_input)) _state_input)
  (:serialization-length
   ()
   (+
    ;; string _state_input
    4 (length _state_input)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _state_input
       (write-long (length _state_input) s) (princ _state_input s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _state_input
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state_input (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tiago1::robotstatedecisionResponse
  :super ros::object
  :slots (_state_output _order _success ))

(defmethod tiago1::robotstatedecisionResponse
  (:init
   (&key
    ((:state_output __state_output) "")
    ((:order __order) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _state_output (string __state_output))
   (setq _order __order)
   (setq _success __success)
   self)
  (:state_output
   (&optional __state_output)
   (if __state_output (setq _state_output __state_output)) _state_output)
  (:order
   (&optional __order)
   (if __order (setq _order __order)) _order)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; string _state_output
    4 (length _state_output)
    ;; string[] _order
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _order)) 4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _state_output
       (write-long (length _state_output) s) (princ _state_output s)
     ;; string[] _order
     (write-long (length _order) s)
     (dolist (elem _order)
       (write-long (length elem) s) (princ elem s)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _state_output
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state_output (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _order
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _order (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _order i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tiago1::robotstatedecision
  :super ros::object
  :slots ())

(setf (get tiago1::robotstatedecision :md5sum-) "26c301d5da14bdddfe3b9fdfb6c1eca2")
(setf (get tiago1::robotstatedecision :datatype-) "tiago1/robotstatedecision")
(setf (get tiago1::robotstatedecision :request) tiago1::robotstatedecisionRequest)
(setf (get tiago1::robotstatedecision :response) tiago1::robotstatedecisionResponse)

(defmethod tiago1::robotstatedecisionRequest
  (:response () (instance tiago1::robotstatedecisionResponse :init)))

(setf (get tiago1::robotstatedecisionRequest :md5sum-) "26c301d5da14bdddfe3b9fdfb6c1eca2")
(setf (get tiago1::robotstatedecisionRequest :datatype-) "tiago1/robotstatedecisionRequest")
(setf (get tiago1::robotstatedecisionRequest :definition-)
      "string state_input 
---
string state_output
string[] order
bool success



")

(setf (get tiago1::robotstatedecisionResponse :md5sum-) "26c301d5da14bdddfe3b9fdfb6c1eca2")
(setf (get tiago1::robotstatedecisionResponse :datatype-) "tiago1/robotstatedecisionResponse")
(setf (get tiago1::robotstatedecisionResponse :definition-)
      "string state_input 
---
string state_output
string[] order
bool success



")



(provide :tiago1/robotstatedecision "26c301d5da14bdddfe3b9fdfb6c1eca2")


