; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GripperConfig_SafetyIdentifier.msg.html

(cl:defclass <GripperConfig_SafetyIdentifier> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GripperConfig_SafetyIdentifier (<GripperConfig_SafetyIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperConfig_SafetyIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperConfig_SafetyIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GripperConfig_SafetyIdentifier> is deprecated: use kortex_driver-msg:GripperConfig_SafetyIdentifier instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GripperConfig_SafetyIdentifier>)))
    "Constants for message type '<GripperConfig_SafetyIdentifier>"
  '((:UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER . 0)
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
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GripperConfig_SafetyIdentifier)))
    "Constants for message type 'GripperConfig_SafetyIdentifier"
  '((:UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER . 0)
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
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperConfig_SafetyIdentifier>) ostream)
  "Serializes a message object of type '<GripperConfig_SafetyIdentifier>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperConfig_SafetyIdentifier>) istream)
  "Deserializes a message object of type '<GripperConfig_SafetyIdentifier>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperConfig_SafetyIdentifier>)))
  "Returns string type for a message object of type '<GripperConfig_SafetyIdentifier>"
  "kortex_driver/GripperConfig_SafetyIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperConfig_SafetyIdentifier)))
  "Returns string type for a message object of type 'GripperConfig_SafetyIdentifier"
  "kortex_driver/GripperConfig_SafetyIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperConfig_SafetyIdentifier>)))
  "Returns md5sum for a message object of type '<GripperConfig_SafetyIdentifier>"
  "d7392d34e7992e1d68eccf5c1f5cd7f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperConfig_SafetyIdentifier)))
  "Returns md5sum for a message object of type 'GripperConfig_SafetyIdentifier"
  "d7392d34e7992e1d68eccf5c1f5cd7f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperConfig_SafetyIdentifier>)))
  "Returns full string definition for message of type '<GripperConfig_SafetyIdentifier>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER = 0~%~%uint32 MAXIMUM_MOTOR_CURRENT = 1~%~%uint32 MAXIMUM_VOLTAGE = 2~%~%uint32 MINIMUM_VOLTAGE = 4~%~%uint32 MAXIMUM_MOTOR_TEMPERATURE = 8~%~%uint32 MAXIMUM_CORE_TEMPERATURE = 16~%~%uint32 NON_VOLATILE_MEMORY_CORRUPTED = 32~%~%uint32 EMERGENCY_LINE_ASSERTED = 64~%~%uint32 COMMUNICATION_TICK_LOST = 128~%~%uint32 WATCHDOG_TRIGGERED = 256~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperConfig_SafetyIdentifier)))
  "Returns full string definition for message of type 'GripperConfig_SafetyIdentifier"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER = 0~%~%uint32 MAXIMUM_MOTOR_CURRENT = 1~%~%uint32 MAXIMUM_VOLTAGE = 2~%~%uint32 MINIMUM_VOLTAGE = 4~%~%uint32 MAXIMUM_MOTOR_TEMPERATURE = 8~%~%uint32 MAXIMUM_CORE_TEMPERATURE = 16~%~%uint32 NON_VOLATILE_MEMORY_CORRUPTED = 32~%~%uint32 EMERGENCY_LINE_ASSERTED = 64~%~%uint32 COMMUNICATION_TICK_LOST = 128~%~%uint32 WATCHDOG_TRIGGERED = 256~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperConfig_SafetyIdentifier>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperConfig_SafetyIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperConfig_SafetyIdentifier
))
