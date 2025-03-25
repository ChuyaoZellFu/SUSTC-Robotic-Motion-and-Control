; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DeviceType.msg.html

(cl:defclass <DeviceType> (roslisp-msg-protocol:ros-message)
  ((device_type
    :reader device_type
    :initarg :device_type
    :type cl:integer
    :initform 0))
)

(cl:defclass DeviceType (<DeviceType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DeviceType> is deprecated: use kortex_driver-msg:DeviceType instead.")))

(cl:ensure-generic-function 'device_type-val :lambda-list '(m))
(cl:defmethod device_type-val ((m <DeviceType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_type-val is deprecated.  Use kortex_driver-msg:device_type instead.")
  (device_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceType>) ostream)
  "Serializes a message object of type '<DeviceType>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceType>) istream)
  "Deserializes a message object of type '<DeviceType>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceType>)))
  "Returns string type for a message object of type '<DeviceType>"
  "kortex_driver/DeviceType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceType)))
  "Returns string type for a message object of type 'DeviceType"
  "kortex_driver/DeviceType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceType>)))
  "Returns md5sum for a message object of type '<DeviceType>"
  "03c69ac8f4cbd0c35678776a27f8d84a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceType)))
  "Returns md5sum for a message object of type 'DeviceType"
  "03c69ac8f4cbd0c35678776a27f8d84a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceType>)))
  "Returns full string definition for message of type '<DeviceType>"
  (cl:format cl:nil "~%uint32 device_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceType)))
  "Returns full string definition for message of type 'DeviceType"
  (cl:format cl:nil "~%uint32 device_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceType>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceType>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceType
    (cl:cons ':device_type (device_type msg))
))
