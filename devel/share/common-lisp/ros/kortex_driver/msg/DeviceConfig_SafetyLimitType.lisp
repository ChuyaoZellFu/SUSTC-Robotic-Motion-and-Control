; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DeviceConfig_SafetyLimitType.msg.html

(cl:defclass <DeviceConfig_SafetyLimitType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DeviceConfig_SafetyLimitType (<DeviceConfig_SafetyLimitType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceConfig_SafetyLimitType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceConfig_SafetyLimitType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DeviceConfig_SafetyLimitType> is deprecated: use kortex_driver-msg:DeviceConfig_SafetyLimitType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DeviceConfig_SafetyLimitType>)))
    "Constants for message type '<DeviceConfig_SafetyLimitType>"
  '((:UNSPECIFIED_SAFETY_LIMIT_TYPE . 0)
    (:MINIMAL_LIMIT . 1)
    (:MAXIMAL_LIMIT . 2)
    (:EVENT_LIMIT . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DeviceConfig_SafetyLimitType)))
    "Constants for message type 'DeviceConfig_SafetyLimitType"
  '((:UNSPECIFIED_SAFETY_LIMIT_TYPE . 0)
    (:MINIMAL_LIMIT . 1)
    (:MAXIMAL_LIMIT . 2)
    (:EVENT_LIMIT . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceConfig_SafetyLimitType>) ostream)
  "Serializes a message object of type '<DeviceConfig_SafetyLimitType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceConfig_SafetyLimitType>) istream)
  "Deserializes a message object of type '<DeviceConfig_SafetyLimitType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceConfig_SafetyLimitType>)))
  "Returns string type for a message object of type '<DeviceConfig_SafetyLimitType>"
  "kortex_driver/DeviceConfig_SafetyLimitType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConfig_SafetyLimitType)))
  "Returns string type for a message object of type 'DeviceConfig_SafetyLimitType"
  "kortex_driver/DeviceConfig_SafetyLimitType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceConfig_SafetyLimitType>)))
  "Returns md5sum for a message object of type '<DeviceConfig_SafetyLimitType>"
  "497794fa9cad838eb83949e4e871d8e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceConfig_SafetyLimitType)))
  "Returns md5sum for a message object of type 'DeviceConfig_SafetyLimitType"
  "497794fa9cad838eb83949e4e871d8e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceConfig_SafetyLimitType>)))
  "Returns full string definition for message of type '<DeviceConfig_SafetyLimitType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SAFETY_LIMIT_TYPE = 0~%~%uint32 MINIMAL_LIMIT = 1~%~%uint32 MAXIMAL_LIMIT = 2~%~%uint32 EVENT_LIMIT = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceConfig_SafetyLimitType)))
  "Returns full string definition for message of type 'DeviceConfig_SafetyLimitType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SAFETY_LIMIT_TYPE = 0~%~%uint32 MINIMAL_LIMIT = 1~%~%uint32 MAXIMAL_LIMIT = 2~%~%uint32 EVENT_LIMIT = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceConfig_SafetyLimitType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceConfig_SafetyLimitType>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceConfig_SafetyLimitType
))
