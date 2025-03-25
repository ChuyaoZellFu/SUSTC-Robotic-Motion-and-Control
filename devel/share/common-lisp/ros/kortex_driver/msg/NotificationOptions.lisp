; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude NotificationOptions.msg.html

(cl:defclass <NotificationOptions> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (rate_m_sec
    :reader rate_m_sec
    :initarg :rate_m_sec
    :type cl:integer
    :initform 0)
   (threshold_value
    :reader threshold_value
    :initarg :threshold_value
    :type cl:float
    :initform 0.0))
)

(cl:defclass NotificationOptions (<NotificationOptions>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NotificationOptions>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NotificationOptions)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<NotificationOptions> is deprecated: use kortex_driver-msg:NotificationOptions instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <NotificationOptions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:type-val is deprecated.  Use kortex_driver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'rate_m_sec-val :lambda-list '(m))
(cl:defmethod rate_m_sec-val ((m <NotificationOptions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:rate_m_sec-val is deprecated.  Use kortex_driver-msg:rate_m_sec instead.")
  (rate_m_sec m))

(cl:ensure-generic-function 'threshold_value-val :lambda-list '(m))
(cl:defmethod threshold_value-val ((m <NotificationOptions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:threshold_value-val is deprecated.  Use kortex_driver-msg:threshold_value instead.")
  (threshold_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NotificationOptions>) ostream)
  "Serializes a message object of type '<NotificationOptions>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rate_m_sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rate_m_sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'rate_m_sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'rate_m_sec)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'threshold_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NotificationOptions>) istream)
  "Deserializes a message object of type '<NotificationOptions>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rate_m_sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rate_m_sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'rate_m_sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'rate_m_sec)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'threshold_value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NotificationOptions>)))
  "Returns string type for a message object of type '<NotificationOptions>"
  "kortex_driver/NotificationOptions")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NotificationOptions)))
  "Returns string type for a message object of type 'NotificationOptions"
  "kortex_driver/NotificationOptions")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NotificationOptions>)))
  "Returns md5sum for a message object of type '<NotificationOptions>"
  "756dde28efcdf67e7515ead30453e781")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NotificationOptions)))
  "Returns md5sum for a message object of type 'NotificationOptions"
  "756dde28efcdf67e7515ead30453e781")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NotificationOptions>)))
  "Returns full string definition for message of type '<NotificationOptions>"
  (cl:format cl:nil "~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NotificationOptions)))
  "Returns full string definition for message of type 'NotificationOptions"
  (cl:format cl:nil "~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NotificationOptions>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NotificationOptions>))
  "Converts a ROS message object to a list"
  (cl:list 'NotificationOptions
    (cl:cons ':type (type msg))
    (cl:cons ':rate_m_sec (rate_m_sec msg))
    (cl:cons ':threshold_value (threshold_value msg))
))
