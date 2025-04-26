; Auto-generated. Do not edit!


(cl:in-package tiago1-srv)


;//! \htmlinclude send_order-request.msg.html

(cl:defclass <send_order-request> (roslisp-msg-protocol:ros-message)
  ((order
    :reader order
    :initarg :order
    :type tiago1-msg:Voice_rec
    :initform (cl:make-instance 'tiago1-msg:Voice_rec))
   (robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:string
    :initform ""))
)

(cl:defclass send_order-request (<send_order-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <send_order-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'send_order-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago1-srv:<send_order-request> is deprecated: use tiago1-srv:send_order-request instead.")))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <send_order-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:order-val is deprecated.  Use tiago1-srv:order instead.")
  (order m))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <send_order-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:robot_id-val is deprecated.  Use tiago1-srv:robot_id instead.")
  (robot_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <send_order-request>) ostream)
  "Serializes a message object of type '<send_order-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'order) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <send_order-request>) istream)
  "Deserializes a message object of type '<send_order-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'order) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<send_order-request>)))
  "Returns string type for a service object of type '<send_order-request>"
  "tiago1/send_orderRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'send_order-request)))
  "Returns string type for a service object of type 'send_order-request"
  "tiago1/send_orderRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<send_order-request>)))
  "Returns md5sum for a message object of type '<send_order-request>"
  "54d692335fd48f1476d58fcf2615595d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'send_order-request)))
  "Returns md5sum for a message object of type 'send_order-request"
  "54d692335fd48f1476d58fcf2615595d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<send_order-request>)))
  "Returns full string definition for message of type '<send_order-request>"
  (cl:format cl:nil "tiago1/Voice_rec order~%string robot_id~%~%================================================================================~%MSG: tiago1/Voice_rec~%int32 id_client~%string[] list_of_orders~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'send_order-request)))
  "Returns full string definition for message of type 'send_order-request"
  (cl:format cl:nil "tiago1/Voice_rec order~%string robot_id~%~%================================================================================~%MSG: tiago1/Voice_rec~%int32 id_client~%string[] list_of_orders~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <send_order-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'order))
     4 (cl:length (cl:slot-value msg 'robot_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <send_order-request>))
  "Converts a ROS message object to a list"
  (cl:list 'send_order-request
    (cl:cons ':order (order msg))
    (cl:cons ':robot_id (robot_id msg))
))
;//! \htmlinclude send_order-response.msg.html

(cl:defclass <send_order-response> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass send_order-response (<send_order-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <send_order-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'send_order-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago1-srv:<send_order-response> is deprecated: use tiago1-srv:send_order-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <send_order-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:message-val is deprecated.  Use tiago1-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <send_order-response>) ostream)
  "Serializes a message object of type '<send_order-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <send_order-response>) istream)
  "Deserializes a message object of type '<send_order-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<send_order-response>)))
  "Returns string type for a service object of type '<send_order-response>"
  "tiago1/send_orderResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'send_order-response)))
  "Returns string type for a service object of type 'send_order-response"
  "tiago1/send_orderResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<send_order-response>)))
  "Returns md5sum for a message object of type '<send_order-response>"
  "54d692335fd48f1476d58fcf2615595d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'send_order-response)))
  "Returns md5sum for a message object of type 'send_order-response"
  "54d692335fd48f1476d58fcf2615595d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<send_order-response>)))
  "Returns full string definition for message of type '<send_order-response>"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'send_order-response)))
  "Returns full string definition for message of type 'send_order-response"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <send_order-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <send_order-response>))
  "Converts a ROS message object to a list"
  (cl:list 'send_order-response
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'send_order)))
  'send_order-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'send_order)))
  'send_order-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'send_order)))
  "Returns string type for a service object of type '<send_order>"
  "tiago1/send_order")