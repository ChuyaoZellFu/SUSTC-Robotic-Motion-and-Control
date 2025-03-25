; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TrajectoryErrorElement.msg.html

(cl:defclass <TrajectoryErrorElement> (roslisp-msg-protocol:ros-message)
  ((error_type
    :reader error_type
    :initarg :error_type
    :type cl:integer
    :initform 0)
   (error_identifier
    :reader error_identifier
    :initarg :error_identifier
    :type cl:integer
    :initform 0)
   (error_value
    :reader error_value
    :initarg :error_value
    :type cl:float
    :initform 0.0)
   (min_value
    :reader min_value
    :initarg :min_value
    :type cl:float
    :initform 0.0)
   (max_value
    :reader max_value
    :initarg :max_value
    :type cl:float
    :initform 0.0)
   (index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (waypoint_index
    :reader waypoint_index
    :initarg :waypoint_index
    :type cl:integer
    :initform 0))
)

(cl:defclass TrajectoryErrorElement (<TrajectoryErrorElement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryErrorElement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryErrorElement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TrajectoryErrorElement> is deprecated: use kortex_driver-msg:TrajectoryErrorElement instead.")))

(cl:ensure-generic-function 'error_type-val :lambda-list '(m))
(cl:defmethod error_type-val ((m <TrajectoryErrorElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:error_type-val is deprecated.  Use kortex_driver-msg:error_type instead.")
  (error_type m))

(cl:ensure-generic-function 'error_identifier-val :lambda-list '(m))
(cl:defmethod error_identifier-val ((m <TrajectoryErrorElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:error_identifier-val is deprecated.  Use kortex_driver-msg:error_identifier instead.")
  (error_identifier m))

(cl:ensure-generic-function 'error_value-val :lambda-list '(m))
(cl:defmethod error_value-val ((m <TrajectoryErrorElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:error_value-val is deprecated.  Use kortex_driver-msg:error_value instead.")
  (error_value m))

(cl:ensure-generic-function 'min_value-val :lambda-list '(m))
(cl:defmethod min_value-val ((m <TrajectoryErrorElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:min_value-val is deprecated.  Use kortex_driver-msg:min_value instead.")
  (min_value m))

(cl:ensure-generic-function 'max_value-val :lambda-list '(m))
(cl:defmethod max_value-val ((m <TrajectoryErrorElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:max_value-val is deprecated.  Use kortex_driver-msg:max_value instead.")
  (max_value m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <TrajectoryErrorElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:index-val is deprecated.  Use kortex_driver-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <TrajectoryErrorElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:message-val is deprecated.  Use kortex_driver-msg:message instead.")
  (message m))

(cl:ensure-generic-function 'waypoint_index-val :lambda-list '(m))
(cl:defmethod waypoint_index-val ((m <TrajectoryErrorElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:waypoint_index-val is deprecated.  Use kortex_driver-msg:waypoint_index instead.")
  (waypoint_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryErrorElement>) ostream)
  "Serializes a message object of type '<TrajectoryErrorElement>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'error_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'error_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'error_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'error_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'error_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'error_identifier)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'index)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waypoint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'waypoint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'waypoint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'waypoint_index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryErrorElement>) istream)
  "Deserializes a message object of type '<TrajectoryErrorElement>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'error_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'error_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'error_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'error_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'error_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'error_identifier)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waypoint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'waypoint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'waypoint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'waypoint_index)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryErrorElement>)))
  "Returns string type for a message object of type '<TrajectoryErrorElement>"
  "kortex_driver/TrajectoryErrorElement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryErrorElement)))
  "Returns string type for a message object of type 'TrajectoryErrorElement"
  "kortex_driver/TrajectoryErrorElement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryErrorElement>)))
  "Returns md5sum for a message object of type '<TrajectoryErrorElement>"
  "fe19f45de474fe8a7f3de643ea88e472")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryErrorElement)))
  "Returns md5sum for a message object of type 'TrajectoryErrorElement"
  "fe19f45de474fe8a7f3de643ea88e472")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryErrorElement>)))
  "Returns full string definition for message of type '<TrajectoryErrorElement>"
  (cl:format cl:nil "~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryErrorElement)))
  "Returns full string definition for message of type 'TrajectoryErrorElement"
  (cl:format cl:nil "~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryErrorElement>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'message))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryErrorElement>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryErrorElement
    (cl:cons ':error_type (error_type msg))
    (cl:cons ':error_identifier (error_identifier msg))
    (cl:cons ':error_value (error_value msg))
    (cl:cons ':min_value (min_value msg))
    (cl:cons ':max_value (max_value msg))
    (cl:cons ':index (index msg))
    (cl:cons ':message (message msg))
    (cl:cons ':waypoint_index (waypoint_index msg))
))
