; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DeviceConfig_CapSenseMode.msg.html

(cl:defclass <DeviceConfig_CapSenseMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DeviceConfig_CapSenseMode (<DeviceConfig_CapSenseMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceConfig_CapSenseMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceConfig_CapSenseMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DeviceConfig_CapSenseMode> is deprecated: use kortex_driver-msg:DeviceConfig_CapSenseMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DeviceConfig_CapSenseMode>)))
    "Constants for message type '<DeviceConfig_CapSenseMode>"
  '((:RESERVED . 0)
    (:INACTIVE . 1)
    (:ACTIVE_AUTO_THRESHOLD . 2)
    (:ACTIVE_NOISE_ATT . 4)
    (:ACTIVE_NORMAL . 5)
    (:CONFIGURATION . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DeviceConfig_CapSenseMode)))
    "Constants for message type 'DeviceConfig_CapSenseMode"
  '((:RESERVED . 0)
    (:INACTIVE . 1)
    (:ACTIVE_AUTO_THRESHOLD . 2)
    (:ACTIVE_NOISE_ATT . 4)
    (:ACTIVE_NORMAL . 5)
    (:CONFIGURATION . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceConfig_CapSenseMode>) ostream)
  "Serializes a message object of type '<DeviceConfig_CapSenseMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceConfig_CapSenseMode>) istream)
  "Deserializes a message object of type '<DeviceConfig_CapSenseMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceConfig_CapSenseMode>)))
  "Returns string type for a message object of type '<DeviceConfig_CapSenseMode>"
  "kortex_driver/DeviceConfig_CapSenseMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConfig_CapSenseMode)))
  "Returns string type for a message object of type 'DeviceConfig_CapSenseMode"
  "kortex_driver/DeviceConfig_CapSenseMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceConfig_CapSenseMode>)))
  "Returns md5sum for a message object of type '<DeviceConfig_CapSenseMode>"
  "ab37ae8a57a132b00bf1d50bed534551")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceConfig_CapSenseMode)))
  "Returns md5sum for a message object of type 'DeviceConfig_CapSenseMode"
  "ab37ae8a57a132b00bf1d50bed534551")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceConfig_CapSenseMode>)))
  "Returns full string definition for message of type '<DeviceConfig_CapSenseMode>"
  (cl:format cl:nil "~%uint32 RESERVED = 0~%~%uint32 INACTIVE = 1~%~%uint32 ACTIVE_AUTO_THRESHOLD = 2~%~%uint32 ACTIVE_NOISE_ATT = 4~%~%uint32 ACTIVE_NORMAL = 5~%~%uint32 CONFIGURATION = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceConfig_CapSenseMode)))
  "Returns full string definition for message of type 'DeviceConfig_CapSenseMode"
  (cl:format cl:nil "~%uint32 RESERVED = 0~%~%uint32 INACTIVE = 1~%~%uint32 ACTIVE_AUTO_THRESHOLD = 2~%~%uint32 ACTIVE_NOISE_ATT = 4~%~%uint32 ACTIVE_NORMAL = 5~%~%uint32 CONFIGURATION = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceConfig_CapSenseMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceConfig_CapSenseMode>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceConfig_CapSenseMode
))
