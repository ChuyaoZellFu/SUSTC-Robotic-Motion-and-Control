; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CDeviceIdentification.msg.html

(cl:defclass <I2CDeviceIdentification> (roslisp-msg-protocol:ros-message)
  ((device
    :reader device
    :initarg :device
    :type cl:integer
    :initform 0))
)

(cl:defclass I2CDeviceIdentification (<I2CDeviceIdentification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CDeviceIdentification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CDeviceIdentification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CDeviceIdentification> is deprecated: use kortex_driver-msg:I2CDeviceIdentification instead.")))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <I2CDeviceIdentification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device-val is deprecated.  Use kortex_driver-msg:device instead.")
  (device m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CDeviceIdentification>) ostream)
  "Serializes a message object of type '<I2CDeviceIdentification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CDeviceIdentification>) istream)
  "Deserializes a message object of type '<I2CDeviceIdentification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CDeviceIdentification>)))
  "Returns string type for a message object of type '<I2CDeviceIdentification>"
  "kortex_driver/I2CDeviceIdentification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CDeviceIdentification)))
  "Returns string type for a message object of type 'I2CDeviceIdentification"
  "kortex_driver/I2CDeviceIdentification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CDeviceIdentification>)))
  "Returns md5sum for a message object of type '<I2CDeviceIdentification>"
  "e97054e4ab2eb3665270f077c46482df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CDeviceIdentification)))
  "Returns md5sum for a message object of type 'I2CDeviceIdentification"
  "e97054e4ab2eb3665270f077c46482df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CDeviceIdentification>)))
  "Returns full string definition for message of type '<I2CDeviceIdentification>"
  (cl:format cl:nil "~%uint32 device~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CDeviceIdentification)))
  "Returns full string definition for message of type 'I2CDeviceIdentification"
  (cl:format cl:nil "~%uint32 device~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CDeviceIdentification>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CDeviceIdentification>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CDeviceIdentification
    (cl:cons ':device (device msg))
))
