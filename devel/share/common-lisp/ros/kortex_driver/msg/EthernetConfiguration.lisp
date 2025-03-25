; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EthernetConfiguration.msg.html

(cl:defclass <EthernetConfiguration> (roslisp-msg-protocol:ros-message)
  ((device
    :reader device
    :initarg :device
    :type cl:integer
    :initform 0)
   (enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (duplex
    :reader duplex
    :initarg :duplex
    :type cl:integer
    :initform 0))
)

(cl:defclass EthernetConfiguration (<EthernetConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EthernetConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EthernetConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EthernetConfiguration> is deprecated: use kortex_driver-msg:EthernetConfiguration instead.")))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <EthernetConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device-val is deprecated.  Use kortex_driver-msg:device instead.")
  (device m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <EthernetConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enabled-val is deprecated.  Use kortex_driver-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <EthernetConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:speed-val is deprecated.  Use kortex_driver-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'duplex-val :lambda-list '(m))
(cl:defmethod duplex-val ((m <EthernetConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duplex-val is deprecated.  Use kortex_driver-msg:duplex instead.")
  (duplex m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EthernetConfiguration>) ostream)
  "Serializes a message object of type '<EthernetConfiguration>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duplex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duplex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duplex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duplex)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EthernetConfiguration>) istream)
  "Deserializes a message object of type '<EthernetConfiguration>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duplex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duplex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duplex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duplex)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EthernetConfiguration>)))
  "Returns string type for a message object of type '<EthernetConfiguration>"
  "kortex_driver/EthernetConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EthernetConfiguration)))
  "Returns string type for a message object of type 'EthernetConfiguration"
  "kortex_driver/EthernetConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EthernetConfiguration>)))
  "Returns md5sum for a message object of type '<EthernetConfiguration>"
  "f0157b8e3c3c47a25a98debdeb63bdb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EthernetConfiguration)))
  "Returns md5sum for a message object of type 'EthernetConfiguration"
  "f0157b8e3c3c47a25a98debdeb63bdb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EthernetConfiguration>)))
  "Returns full string definition for message of type '<EthernetConfiguration>"
  (cl:format cl:nil "~%uint32 device~%bool enabled~%uint32 speed~%uint32 duplex~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EthernetConfiguration)))
  "Returns full string definition for message of type 'EthernetConfiguration"
  (cl:format cl:nil "~%uint32 device~%bool enabled~%uint32 speed~%uint32 duplex~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EthernetConfiguration>))
  (cl:+ 0
     4
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EthernetConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'EthernetConfiguration
    (cl:cons ':device (device msg))
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':duplex (duplex msg))
))
