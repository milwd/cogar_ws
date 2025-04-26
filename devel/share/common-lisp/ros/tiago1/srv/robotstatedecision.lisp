; Auto-generated. Do not edit!


(cl:in-package tiago1-srv)


;//! \htmlinclude robotstatedecision-request.msg.html

(cl:defclass <robotstatedecision-request> (roslisp-msg-protocol:ros-message)
  ((state_input
    :reader state_input
    :initarg :state_input
    :type cl:string
    :initform "")
   (robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:string
    :initform ""))
)

(cl:defclass robotstatedecision-request (<robotstatedecision-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robotstatedecision-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robotstatedecision-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago1-srv:<robotstatedecision-request> is deprecated: use tiago1-srv:robotstatedecision-request instead.")))

(cl:ensure-generic-function 'state_input-val :lambda-list '(m))
(cl:defmethod state_input-val ((m <robotstatedecision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:state_input-val is deprecated.  Use tiago1-srv:state_input instead.")
  (state_input m))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <robotstatedecision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:robot_id-val is deprecated.  Use tiago1-srv:robot_id instead.")
  (robot_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robotstatedecision-request>) ostream)
  "Serializes a message object of type '<robotstatedecision-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state_input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state_input))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robotstatedecision-request>) istream)
  "Deserializes a message object of type '<robotstatedecision-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state_input) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state_input) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robotstatedecision-request>)))
  "Returns string type for a service object of type '<robotstatedecision-request>"
  "tiago1/robotstatedecisionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robotstatedecision-request)))
  "Returns string type for a service object of type 'robotstatedecision-request"
  "tiago1/robotstatedecisionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robotstatedecision-request>)))
  "Returns md5sum for a message object of type '<robotstatedecision-request>"
  "33ec071b72a9a044f86da64824d0e481")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robotstatedecision-request)))
  "Returns md5sum for a message object of type 'robotstatedecision-request"
  "33ec071b72a9a044f86da64824d0e481")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robotstatedecision-request>)))
  "Returns full string definition for message of type '<robotstatedecision-request>"
  (cl:format cl:nil "string state_input ~%string robot_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robotstatedecision-request)))
  "Returns full string definition for message of type 'robotstatedecision-request"
  (cl:format cl:nil "string state_input ~%string robot_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robotstatedecision-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'state_input))
     4 (cl:length (cl:slot-value msg 'robot_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robotstatedecision-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robotstatedecision-request
    (cl:cons ':state_input (state_input msg))
    (cl:cons ':robot_id (robot_id msg))
))
;//! \htmlinclude robotstatedecision-response.msg.html

(cl:defclass <robotstatedecision-response> (roslisp-msg-protocol:ros-message)
  ((state_output
    :reader state_output
    :initarg :state_output
    :type cl:string
    :initform "")
   (id_client
    :reader id_client
    :initarg :id_client
    :type cl:integer
    :initform 0)
   (order
    :reader order
    :initarg :order
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass robotstatedecision-response (<robotstatedecision-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robotstatedecision-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robotstatedecision-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago1-srv:<robotstatedecision-response> is deprecated: use tiago1-srv:robotstatedecision-response instead.")))

(cl:ensure-generic-function 'state_output-val :lambda-list '(m))
(cl:defmethod state_output-val ((m <robotstatedecision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:state_output-val is deprecated.  Use tiago1-srv:state_output instead.")
  (state_output m))

(cl:ensure-generic-function 'id_client-val :lambda-list '(m))
(cl:defmethod id_client-val ((m <robotstatedecision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:id_client-val is deprecated.  Use tiago1-srv:id_client instead.")
  (id_client m))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <robotstatedecision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:order-val is deprecated.  Use tiago1-srv:order instead.")
  (order m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <robotstatedecision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago1-srv:success-val is deprecated.  Use tiago1-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robotstatedecision-response>) ostream)
  "Serializes a message object of type '<robotstatedecision-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state_output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state_output))
  (cl:let* ((signed (cl:slot-value msg 'id_client)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'order))))
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
   (cl:slot-value msg 'order))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robotstatedecision-response>) istream)
  "Deserializes a message object of type '<robotstatedecision-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state_output) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state_output) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
  (cl:setf (cl:slot-value msg 'order) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'order)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robotstatedecision-response>)))
  "Returns string type for a service object of type '<robotstatedecision-response>"
  "tiago1/robotstatedecisionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robotstatedecision-response)))
  "Returns string type for a service object of type 'robotstatedecision-response"
  "tiago1/robotstatedecisionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robotstatedecision-response>)))
  "Returns md5sum for a message object of type '<robotstatedecision-response>"
  "33ec071b72a9a044f86da64824d0e481")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robotstatedecision-response)))
  "Returns md5sum for a message object of type 'robotstatedecision-response"
  "33ec071b72a9a044f86da64824d0e481")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robotstatedecision-response>)))
  "Returns full string definition for message of type '<robotstatedecision-response>"
  (cl:format cl:nil "string state_output~%int32 id_client~%string[] order~%bool success~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robotstatedecision-response)))
  "Returns full string definition for message of type 'robotstatedecision-response"
  (cl:format cl:nil "string state_output~%int32 id_client~%string[] order~%bool success~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robotstatedecision-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'state_output))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'order) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robotstatedecision-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robotstatedecision-response
    (cl:cons ':state_output (state_output msg))
    (cl:cons ':id_client (id_client msg))
    (cl:cons ':order (order msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robotstatedecision)))
  'robotstatedecision-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robotstatedecision)))
  'robotstatedecision-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robotstatedecision)))
  "Returns string type for a service object of type '<robotstatedecision>"
  "tiago1/robotstatedecision")