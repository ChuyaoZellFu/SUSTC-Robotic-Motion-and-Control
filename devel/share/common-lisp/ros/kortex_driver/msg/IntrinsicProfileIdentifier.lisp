; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude IntrinsicProfileIdentifier.msg.html

(cl:defclass <IntrinsicProfileIdentifier> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type cl:integer
    :initform 0)
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:integer
    :initform 0))
)

(cl:defclass IntrinsicProfileIdentifier (<IntrinsicProfileIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntrinsicProfileIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntrinsicProfileIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<IntrinsicProfileIdentifier> is deprecated: use kortex_driver-msg:IntrinsicProfileIdentifier instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <IntrinsicProfileIdentifier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensor-val is deprecated.  Use kortex_driver-msg:sensor instead.")
  (sensor m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <IntrinsicProfileIdentifier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:resolution-val is deprecated.  Use kortex_driver-msg:resolution instead.")
  (resolution m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntrinsicProfileIdentifier>) ostream)
  "Serializes a message object of type '<IntrinsicProfileIdentifier>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'resolution)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntrinsicProfileIdentifier>) istream)
  "Deserializes a message object of type '<IntrinsicProfileIdentifier>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntrinsicProfileIdentifier>)))
  "Returns string type for a message object of type '<IntrinsicProfileIdentifier>"
  "kortex_driver/IntrinsicProfileIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntrinsicProfileIdentifier)))
  "Returns string type for a message object of type 'IntrinsicProfileIdentifier"
  "kortex_driver/IntrinsicProfileIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntrinsicProfileIdentifier>)))
  "Returns md5sum for a message object of type '<IntrinsicProfileIdentifier>"
  "2934edac4ee0b8f88644bc2d0135508a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntrinsicProfileIdentifier)))
  "Returns md5sum for a message object of type 'IntrinsicProfileIdentifier"
  "2934edac4ee0b8f88644bc2d0135508a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntrinsicProfileIdentifier>)))
  "Returns full string definition for message of type '<IntrinsicProfileIdentifier>"
  (cl:format cl:nil "~%uint32 sensor~%uint32 resolution~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntrinsicProfileIdentifier)))
  "Returns full string definition for message of type 'IntrinsicProfileIdentifier"
  (cl:format cl:nil "~%uint32 sensor~%uint32 resolution~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntrinsicProfileIdentifier>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntrinsicProfileIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'IntrinsicProfileIdentifier
    (cl:cons ':sensor (sensor msg))
    (cl:cons ':resolution (resolution msg))
))
