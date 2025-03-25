; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DeviceTypes.msg.html

(cl:defclass <DeviceTypes> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DeviceTypes (<DeviceTypes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceTypes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceTypes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DeviceTypes> is deprecated: use kortex_driver-msg:DeviceTypes instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DeviceTypes>)))
    "Constants for message type '<DeviceTypes>"
  '((:UNSPECIFIED_DEVICE_TYPE . 0)
    (:BASE . 1)
    (:VISION . 2)
    (:BIG_ACTUATOR . 3)
    (:SMALL_ACTUATOR . 4)
    (:INTERCONNECT . 5)
    (:GRIPPER . 6)
    (:MEDIUM_ACTUATOR . 7)
    (:XBIG_ACTUATOR . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DeviceTypes)))
    "Constants for message type 'DeviceTypes"
  '((:UNSPECIFIED_DEVICE_TYPE . 0)
    (:BASE . 1)
    (:VISION . 2)
    (:BIG_ACTUATOR . 3)
    (:SMALL_ACTUATOR . 4)
    (:INTERCONNECT . 5)
    (:GRIPPER . 6)
    (:MEDIUM_ACTUATOR . 7)
    (:XBIG_ACTUATOR . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceTypes>) ostream)
  "Serializes a message object of type '<DeviceTypes>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceTypes>) istream)
  "Deserializes a message object of type '<DeviceTypes>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceTypes>)))
  "Returns string type for a message object of type '<DeviceTypes>"
  "kortex_driver/DeviceTypes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceTypes)))
  "Returns string type for a message object of type 'DeviceTypes"
  "kortex_driver/DeviceTypes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceTypes>)))
  "Returns md5sum for a message object of type '<DeviceTypes>"
  "46c218f31055194b4054a2785d94d7f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceTypes)))
  "Returns md5sum for a message object of type 'DeviceTypes"
  "46c218f31055194b4054a2785d94d7f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceTypes>)))
  "Returns full string definition for message of type '<DeviceTypes>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_DEVICE_TYPE = 0~%~%uint32 BASE = 1~%~%uint32 VISION = 2~%~%uint32 BIG_ACTUATOR = 3~%~%uint32 SMALL_ACTUATOR = 4~%~%uint32 INTERCONNECT = 5~%~%uint32 GRIPPER = 6~%~%uint32 MEDIUM_ACTUATOR = 7~%~%uint32 XBIG_ACTUATOR = 8~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceTypes)))
  "Returns full string definition for message of type 'DeviceTypes"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_DEVICE_TYPE = 0~%~%uint32 BASE = 1~%~%uint32 VISION = 2~%~%uint32 BIG_ACTUATOR = 3~%~%uint32 SMALL_ACTUATOR = 4~%~%uint32 INTERCONNECT = 5~%~%uint32 GRIPPER = 6~%~%uint32 MEDIUM_ACTUATOR = 7~%~%uint32 XBIG_ACTUATOR = 8~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceTypes>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceTypes>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceTypes
))
