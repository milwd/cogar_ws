;; Auto-generated. Do not edit!


(when (boundp 'tiago1::Voice_rec)
  (if (not (find-package "TIAGO1"))
    (make-package "TIAGO1"))
  (shadow 'Voice_rec (find-package "TIAGO1")))
(unless (find-package "TIAGO1::VOICE_REC")
  (make-package "TIAGO1::VOICE_REC"))

(in-package "ROS")
;;//! \htmlinclude Voice_rec.msg.html


(defclass tiago1::Voice_rec
  :super ros::object
  :slots (_id_client _list_of_orders ))

(defmethod tiago1::Voice_rec
  (:init
   (&key
    ((:id_client __id_client) 0)
    ((:list_of_orders __list_of_orders) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _id_client (round __id_client))
   (setq _list_of_orders __list_of_orders)
   self)
  (:id_client
   (&optional __id_client)
   (if __id_client (setq _id_client __id_client)) _id_client)
  (:list_of_orders
   (&optional __list_of_orders)
   (if __list_of_orders (setq _list_of_orders __list_of_orders)) _list_of_orders)
  (:serialization-length
   ()
   (+
    ;; int32 _id_client
    4
    ;; string[] _list_of_orders
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _list_of_orders)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id_client
       (write-long _id_client s)
     ;; string[] _list_of_orders
     (write-long (length _list_of_orders) s)
     (dolist (elem _list_of_orders)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id_client
     (setq _id_client (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _list_of_orders
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _list_of_orders (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _list_of_orders i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get tiago1::Voice_rec :md5sum-) "e4a95f50c96541db33e3cde77c3a65f4")
(setf (get tiago1::Voice_rec :datatype-) "tiago1/Voice_rec")
(setf (get tiago1::Voice_rec :definition-)
      "int32 id_client
string[] list_of_orders
")



(provide :tiago1/Voice_rec "e4a95f50c96541db33e3cde77c3a65f4")


