;; Auto-generated. Do not edit!


(when (boundp 'tiago1::send_order)
  (if (not (find-package "TIAGO1"))
    (make-package "TIAGO1"))
  (shadow 'send_order (find-package "TIAGO1")))
(unless (find-package "TIAGO1::SEND_ORDER")
  (make-package "TIAGO1::SEND_ORDER"))
(unless (find-package "TIAGO1::SEND_ORDERREQUEST")
  (make-package "TIAGO1::SEND_ORDERREQUEST"))
(unless (find-package "TIAGO1::SEND_ORDERRESPONSE")
  (make-package "TIAGO1::SEND_ORDERRESPONSE"))

(in-package "ROS")





(defclass tiago1::send_orderRequest
  :super ros::object
  :slots (_order ))

(defmethod tiago1::send_orderRequest
  (:init
   (&key
    ((:order __order) (instance tiago1::Voice_rec :init))
    )
   (send-super :init)
   (setq _order __order)
   self)
  (:order
   (&rest __order)
   (if (keywordp (car __order))
       (send* _order __order)
     (progn
       (if __order (setq _order (car __order)))
       _order)))
  (:serialization-length
   ()
   (+
    ;; tiago1/Voice_rec _order
    (send _order :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; tiago1/Voice_rec _order
       (send _order :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; tiago1/Voice_rec _order
     (send _order :deserialize buf ptr-) (incf ptr- (send _order :serialization-length))
   ;;
   self)
  )

(defclass tiago1::send_orderResponse
  :super ros::object
  :slots (_message ))

(defmethod tiago1::send_orderResponse
  (:init
   (&key
    ((:message __message) "")
    )
   (send-super :init)
   (setq _message (string __message))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tiago1::send_order
  :super ros::object
  :slots ())

(setf (get tiago1::send_order :md5sum-) "741481bb0a1ea48957f004257c6bcdd4")
(setf (get tiago1::send_order :datatype-) "tiago1/send_order")
(setf (get tiago1::send_order :request) tiago1::send_orderRequest)
(setf (get tiago1::send_order :response) tiago1::send_orderResponse)

(defmethod tiago1::send_orderRequest
  (:response () (instance tiago1::send_orderResponse :init)))

(setf (get tiago1::send_orderRequest :md5sum-) "741481bb0a1ea48957f004257c6bcdd4")
(setf (get tiago1::send_orderRequest :datatype-) "tiago1/send_orderRequest")
(setf (get tiago1::send_orderRequest :definition-)
      "tiago1/Voice_rec order

================================================================================
MSG: tiago1/Voice_rec
int32 id_client
---
string message
")

(setf (get tiago1::send_orderResponse :md5sum-) "741481bb0a1ea48957f004257c6bcdd4")
(setf (get tiago1::send_orderResponse :datatype-) "tiago1/send_orderResponse")
(setf (get tiago1::send_orderResponse :definition-)
      "tiago1/Voice_rec order

================================================================================
MSG: tiago1/Voice_rec
int32 id_client
---
string message
")



(provide :tiago1/send_order "741481bb0a1ea48957f004257c6bcdd4")


