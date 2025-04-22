; Auto-generated. Do not edit!


(cl:in-package tiago1-msg)


;//! \htmlinclude Voice_rec.msg.html

(cl:defclass <Voice_rec> (roslisp-msg-protocol:ros-message)
  ((id_client
    :reader id_client
    :initarg :id_client
    :type cl:integer
    :initform 0)
   (list_of_orders
    :reader list_of_orders
    :initarg :list_of_orders
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Voice_rec (<Voice_rec>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Voice_rec>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Voice_rec)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago1-msg:<Voice_rec> is deprecated: use tiago1-msg:Voice_rec instead.")))

(cl:ensure-generic-function 'id_client-val :lambda-list '(m))
(cl:defmethod id_client-val ((m <Voice_rec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-msg:id_client-val is deprecated.  Use tiago1-msg:id_client instead.")
  (id_client m))

(cl:ensure-generic-function 'list_of_orders-val :lambda-list '(m))
(cl:defmethod list_of_orders-val ((m <Voice_rec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-msg:list_of_orders-val is deprecated.  Use tiago1-msg:list_of_orders instead.")
  (list_of_orders m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Voice_rec>) ostream)
  "Serializes a message object of type '<Voice_rec>"
  (cl:let* ((signed (cl:slot-value msg 'id_client)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'list_of_orders))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'list_of_orders))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Voice_rec>) istream)
  "Deserializes a message object of type '<Voice_rec>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id_client) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'list_of_orders) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'list_of_orders)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Voice_rec>)))
  "Returns string type for a message object of type '<Voice_rec>"
  "tiago1/Voice_rec")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Voice_rec)))
  "Returns string type for a message object of type 'Voice_rec"
  "tiago1/Voice_rec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Voice_rec>)))
  "Returns md5sum for a message object of type '<Voice_rec>"
  "e4a95f50c96541db33e3cde77c3a65f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Voice_rec)))
  "Returns md5sum for a message object of type 'Voice_rec"
  "e4a95f50c96541db33e3cde77c3a65f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Voice_rec>)))
  "Returns full string definition for message of type '<Voice_rec>"
  (cl:format cl:nil "int32 id_client~%string[] list_of_orders~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Voice_rec)))
  "Returns full string definition for message of type 'Voice_rec"
  (cl:format cl:nil "int32 id_client~%string[] list_of_orders~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Voice_rec>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'list_of_orders) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Voice_rec>))
  "Converts a ROS message object to a list"
  (cl:list 'Voice_rec
    (cl:cons ':id_client (id_client msg))
    (cl:cons ':list_of_orders (list_of_orders msg))
))
