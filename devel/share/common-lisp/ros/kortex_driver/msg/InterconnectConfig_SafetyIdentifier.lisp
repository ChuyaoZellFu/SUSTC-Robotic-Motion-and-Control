; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude InterconnectConfig_SafetyIdentifier.msg.html

(cl:defclass <InterconnectConfig_SafetyIdentifier> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass InterconnectConfig_SafetyIdentifier (<InterconnectConfig_SafetyIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterconnectConfig_SafetyIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterconnectConfig_SafetyIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<InterconnectConfig_SafetyIdentifier> is deprecated: use kortex_driver-msg:InterconnectConfig_SafetyIdentifier instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<InterconnectConfig_SafetyIdentifier>)))
    "Constants for message type '<InterconnectConfig_SafetyIdentifier>"
  '((:UNSPECIFIED_INTERCONNECT_SAFETY_IDENTIFIER . 0)
    (:MAXIMUM_MOTOR_CURRENT . 1)
    (:MAXIMUM_VOLTAGE . 2)
    (:MINIMUM_VOLTAGE . 4)
    (:MAXIMUM_MOTOR_TEMPERATURE . 8)
    (:MAXIMUM_CORE_TEMPERATURE . 16)
    (:NON_VOLATILE_MEMORY_CORRUPTED . 32)
    (:EMERGENCY_LINE_ASSERTED . 64)
    (:COMMUNICATION_TICK_LOST . 128)
    (:WATCHDOG_TRIGGERED . 256))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'InterconnectConfig_SafetyIdentifier)))
    "Constants for message type 'InterconnectConfig_SafetyIdentifier"
  '((:UNSPECIFIED_INTERCONNECT_SAFETY_IDENTIFIER . 0)
    (:MAXIMUM_MOTOR_CURRENT . 1)
    (:MAXIMUM_VOLTAGE . 2)
    (:MINIMUM_VOLTAGE . 4)
    (:MAXIMUM_MOTOR_TEMPERATURE . 8)
    (:MAXIMUM_CORE_TEMPERATURE . 16)
    (:NON_VOLATILE_MEMORY_CORRUPTED . 32)
    (:EMERGENCY_LINE_ASSERTED . 64)
    (:COMMUNICATION_TICK_LOST . 128)
    (:WATCHDOG_TRIGGERED . 256))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterconnectConfig_SafetyIdentifier>) ostream)
  "Serializes a message object of type '<InterconnectConfig_SafetyIdentifier>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterconnectConfig_SafetyIdentifier>) istream)
  "Deserializes a message object of type '<InterconnectConfig_SafetyIdentifier>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterconnectConfig_SafetyIdentifier>)))
  "Returns string type for a message object of type '<InterconnectConfig_SafetyIdentifier>"
  "kortex_driver/InterconnectConfig_SafetyIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterconnectConfig_SafetyIdentifier)))
  "Returns string type for a message object of type 'InterconnectConfig_SafetyIdentifier"
  "kortex_driver/InterconnectConfig_SafetyIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterconnectConfig_SafetyIdentifier>)))
  "Returns md5sum for a message object of type '<InterconnectConfig_SafetyIdentifier>"
  "b36683ad678ea1db45ab5eb82312ac0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterconnectConfig_SafetyIdentifier)))
  "Returns md5sum for a message object of type 'InterconnectConfig_SafetyIdentifier"
  "b36683ad678ea1db45ab5eb82312ac0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterconnectConfig_SafetyIdentifier>)))
  "Returns full string definition for message of type '<InterconnectConfig_SafetyIdentifier>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_INTERCONNECT_SAFETY_IDENTIFIER = 0~%~%uint32 MAXIMUM_MOTOR_CURRENT = 1~%~%uint32 MAXIMUM_VOLTAGE = 2~%~%uint32 MINIMUM_VOLTAGE = 4~%~%uint32 MAXIMUM_MOTOR_TEMPERATURE = 8~%~%uint32 MAXIMUM_CORE_TEMPERATURE = 16~%~%uint32 NON_VOLATILE_MEMORY_CORRUPTED = 32~%~%uint32 EMERGENCY_LINE_ASSERTED = 64~%~%uint32 COMMUNICATION_TICK_LOST = 128~%~%uint32 WATCHDOG_TRIGGERED = 256~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterconnectConfig_SafetyIdentifier)))
  "Returns full string definition for message of type 'InterconnectConfig_SafetyIdentifier"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_INTERCONNECT_SAFETY_IDENTIFIER = 0~%~%uint32 MAXIMUM_MOTOR_CURRENT = 1~%~%uint32 MAXIMUM_VOLTAGE = 2~%~%uint32 MINIMUM_VOLTAGE = 4~%~%uint32 MAXIMUM_MOTOR_TEMPERATURE = 8~%~%uint32 MAXIMUM_CORE_TEMPERATURE = 16~%~%uint32 NON_VOLATILE_MEMORY_CORRUPTED = 32~%~%uint32 EMERGENCY_LINE_ASSERTED = 64~%~%uint32 COMMUNICATION_TICK_LOST = 128~%~%uint32 WATCHDOG_TRIGGERED = 256~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterconnectConfig_SafetyIdentifier>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterconnectConfig_SafetyIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'InterconnectConfig_SafetyIdentifier
))
