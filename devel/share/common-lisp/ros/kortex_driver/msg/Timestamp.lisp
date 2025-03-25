; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Timestamp.msg.html

(cl:defclass <Timestamp> (roslisp-msg-protocol:ros-message)
  ((sec
    :reader sec
    :initarg :sec
    :type cl:integer
    :initform 0)
   (usec
    :reader usec
    :initarg :usec
    :type cl:integer
    :initform 0))
)

(cl:defclass Timestamp (<Timestamp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Timestamp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Timestamp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Timestamp> is deprecated: use kortex_driver-msg:Timestamp instead.")))

(cl:ensure-generic-function 'sec-val :lambda-list '(m))
(cl:defmethod sec-val ((m <Timestamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sec-val is deprecated.  Use kortex_driver-msg:sec instead.")
  (sec m))

(cl:ensure-generic-function 'usec-val :lambda-list '(m))
(cl:defmethod usec-val ((m <Timestamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:usec-val is deprecated.  Use kortex_driver-msg:usec instead.")
  (usec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Timestamp>) ostream)
  "Serializes a message object of type '<Timestamp>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'usec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'usec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'usec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'usec)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Timestamp>) istream)
  "Deserializes a message object of type '<Timestamp>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'usec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'usec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'usec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'usec)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Timestamp>)))
  "Returns string type for a message object of type '<Timestamp>"
  "kortex_driver/Timestamp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Timestamp)))
  "Returns string type for a message object of type 'Timestamp"
  "kortex_driver/Timestamp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Timestamp>)))
  "Returns md5sum for a message object of type '<Timestamp>"
  "90da89d9efe7e712be65708e3d776fa4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Timestamp)))
  "Returns md5sum for a message object of type 'Timestamp"
  "90da89d9efe7e712be65708e3d776fa4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Timestamp>)))
  "Returns full string definition for message of type '<Timestamp>"
  (cl:format cl:nil "~%uint32 sec~%uint32 usec~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Timestamp)))
  "Returns full string definition for message of type 'Timestamp"
  (cl:format cl:nil "~%uint32 sec~%uint32 usec~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Timestamp>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Timestamp>))
  "Converts a ROS message object to a list"
  (cl:list 'Timestamp
    (cl:cons ':sec (sec msg))
    (cl:cons ':usec (usec msg))
))
