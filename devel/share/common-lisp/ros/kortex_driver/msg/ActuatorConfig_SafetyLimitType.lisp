; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActuatorConfig_SafetyLimitType.msg.html

(cl:defclass <ActuatorConfig_SafetyLimitType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ActuatorConfig_SafetyLimitType (<ActuatorConfig_SafetyLimitType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorConfig_SafetyLimitType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorConfig_SafetyLimitType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActuatorConfig_SafetyLimitType> is deprecated: use kortex_driver-msg:ActuatorConfig_SafetyLimitType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ActuatorConfig_SafetyLimitType>)))
    "Constants for message type '<ActuatorConfig_SafetyLimitType>"
  '((:MAXIMAL_LIMIT . 0)
    (:MINIMAL_LIMIT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ActuatorConfig_SafetyLimitType)))
    "Constants for message type 'ActuatorConfig_SafetyLimitType"
  '((:MAXIMAL_LIMIT . 0)
    (:MINIMAL_LIMIT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorConfig_SafetyLimitType>) ostream)
  "Serializes a message object of type '<ActuatorConfig_SafetyLimitType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorConfig_SafetyLimitType>) istream)
  "Deserializes a message object of type '<ActuatorConfig_SafetyLimitType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorConfig_SafetyLimitType>)))
  "Returns string type for a message object of type '<ActuatorConfig_SafetyLimitType>"
  "kortex_driver/ActuatorConfig_SafetyLimitType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorConfig_SafetyLimitType)))
  "Returns string type for a message object of type 'ActuatorConfig_SafetyLimitType"
  "kortex_driver/ActuatorConfig_SafetyLimitType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorConfig_SafetyLimitType>)))
  "Returns md5sum for a message object of type '<ActuatorConfig_SafetyLimitType>"
  "2886a0a967849f2573bf0fd798659a83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorConfig_SafetyLimitType)))
  "Returns md5sum for a message object of type 'ActuatorConfig_SafetyLimitType"
  "2886a0a967849f2573bf0fd798659a83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorConfig_SafetyLimitType>)))
  "Returns full string definition for message of type '<ActuatorConfig_SafetyLimitType>"
  (cl:format cl:nil "~%uint32 MAXIMAL_LIMIT = 0~%~%uint32 MINIMAL_LIMIT = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorConfig_SafetyLimitType)))
  "Returns full string definition for message of type 'ActuatorConfig_SafetyLimitType"
  (cl:format cl:nil "~%uint32 MAXIMAL_LIMIT = 0~%~%uint32 MINIMAL_LIMIT = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorConfig_SafetyLimitType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorConfig_SafetyLimitType>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorConfig_SafetyLimitType
))
