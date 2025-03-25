; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SystemTime.msg.html

(cl:defclass <SystemTime> (roslisp-msg-protocol:ros-message)
  ((sec
    :reader sec
    :initarg :sec
    :type cl:integer
    :initform 0)
   (min
    :reader min
    :initarg :min
    :type cl:integer
    :initform 0)
   (hour
    :reader hour
    :initarg :hour
    :type cl:integer
    :initform 0)
   (mday
    :reader mday
    :initarg :mday
    :type cl:integer
    :initform 0)
   (mon
    :reader mon
    :initarg :mon
    :type cl:integer
    :initform 0)
   (year
    :reader year
    :initarg :year
    :type cl:integer
    :initform 0))
)

(cl:defclass SystemTime (<SystemTime>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SystemTime>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SystemTime)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SystemTime> is deprecated: use kortex_driver-msg:SystemTime instead.")))

(cl:ensure-generic-function 'sec-val :lambda-list '(m))
(cl:defmethod sec-val ((m <SystemTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sec-val is deprecated.  Use kortex_driver-msg:sec instead.")
  (sec m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <SystemTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:min-val is deprecated.  Use kortex_driver-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'hour-val :lambda-list '(m))
(cl:defmethod hour-val ((m <SystemTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:hour-val is deprecated.  Use kortex_driver-msg:hour instead.")
  (hour m))

(cl:ensure-generic-function 'mday-val :lambda-list '(m))
(cl:defmethod mday-val ((m <SystemTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mday-val is deprecated.  Use kortex_driver-msg:mday instead.")
  (mday m))

(cl:ensure-generic-function 'mon-val :lambda-list '(m))
(cl:defmethod mon-val ((m <SystemTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mon-val is deprecated.  Use kortex_driver-msg:mon instead.")
  (mon m))

(cl:ensure-generic-function 'year-val :lambda-list '(m))
(cl:defmethod year-val ((m <SystemTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:year-val is deprecated.  Use kortex_driver-msg:year instead.")
  (year m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SystemTime>) ostream)
  "Serializes a message object of type '<SystemTime>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hour)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hour)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hour)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hour)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mday)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mday)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mday)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mday)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mon)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mon)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mon)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mon)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'year)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SystemTime>) istream)
  "Deserializes a message object of type '<SystemTime>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hour)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hour)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hour)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hour)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mday)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mday)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mday)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mday)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mon)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mon)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mon)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mon)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'year)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SystemTime>)))
  "Returns string type for a message object of type '<SystemTime>"
  "kortex_driver/SystemTime")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SystemTime)))
  "Returns string type for a message object of type 'SystemTime"
  "kortex_driver/SystemTime")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SystemTime>)))
  "Returns md5sum for a message object of type '<SystemTime>"
  "c3c3b4e6f22b647897641173820f05db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SystemTime)))
  "Returns md5sum for a message object of type 'SystemTime"
  "c3c3b4e6f22b647897641173820f05db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SystemTime>)))
  "Returns full string definition for message of type '<SystemTime>"
  (cl:format cl:nil "~%uint32 sec~%uint32 min~%uint32 hour~%uint32 mday~%uint32 mon~%uint32 year~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SystemTime)))
  "Returns full string definition for message of type 'SystemTime"
  (cl:format cl:nil "~%uint32 sec~%uint32 min~%uint32 hour~%uint32 mday~%uint32 mon~%uint32 year~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SystemTime>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SystemTime>))
  "Converts a ROS message object to a list"
  (cl:list 'SystemTime
    (cl:cons ':sec (sec msg))
    (cl:cons ':min (min msg))
    (cl:cons ':hour (hour msg))
    (cl:cons ':mday (mday msg))
    (cl:cons ':mon (mon msg))
    (cl:cons ':year (year msg))
))
