; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CConfiguration.msg.html

(cl:defclass <I2CConfiguration> (roslisp-msg-protocol:ros-message)
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
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (addressing
    :reader addressing
    :initarg :addressing
    :type cl:integer
    :initform 0))
)

(cl:defclass I2CConfiguration (<I2CConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CConfiguration> is deprecated: use kortex_driver-msg:I2CConfiguration instead.")))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <I2CConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device-val is deprecated.  Use kortex_driver-msg:device instead.")
  (device m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <I2CConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enabled-val is deprecated.  Use kortex_driver-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <I2CConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mode-val is deprecated.  Use kortex_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'addressing-val :lambda-list '(m))
(cl:defmethod addressing-val ((m <I2CConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:addressing-val is deprecated.  Use kortex_driver-msg:addressing instead.")
  (addressing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CConfiguration>) ostream)
  "Serializes a message object of type '<I2CConfiguration>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addressing)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'addressing)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'addressing)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'addressing)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CConfiguration>) istream)
  "Deserializes a message object of type '<I2CConfiguration>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addressing)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'addressing)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'addressing)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'addressing)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CConfiguration>)))
  "Returns string type for a message object of type '<I2CConfiguration>"
  "kortex_driver/I2CConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CConfiguration)))
  "Returns string type for a message object of type 'I2CConfiguration"
  "kortex_driver/I2CConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CConfiguration>)))
  "Returns md5sum for a message object of type '<I2CConfiguration>"
  "97b0dc0932403bfd636c81257df2603c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CConfiguration)))
  "Returns md5sum for a message object of type 'I2CConfiguration"
  "97b0dc0932403bfd636c81257df2603c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CConfiguration>)))
  "Returns full string definition for message of type '<I2CConfiguration>"
  (cl:format cl:nil "~%uint32 device~%bool enabled~%uint32 mode~%uint32 addressing~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CConfiguration)))
  "Returns full string definition for message of type 'I2CConfiguration"
  (cl:format cl:nil "~%uint32 device~%bool enabled~%uint32 mode~%uint32 addressing~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CConfiguration>))
  (cl:+ 0
     4
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CConfiguration
    (cl:cons ':device (device msg))
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':addressing (addressing msg))
))
