; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlConfigurationEvent.msg.html

(cl:defclass <ControlConfigurationEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControlConfigurationEvent (<ControlConfigurationEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfigurationEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfigurationEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlConfigurationEvent> is deprecated: use kortex_driver-msg:ControlConfigurationEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlConfigurationEvent>)))
    "Constants for message type '<ControlConfigurationEvent>"
  '((:UNSPECIFIED_CONTROL_CONFIGURATION_EVENT . 0)
    (:ANGLE_UNIT_CHANGED . 1)
    (:GRAVITY_VECTOR_CHANGED . 2)
    (:JOINT_ADMITTANCE_CONFIGURATION_CHANGED . 4)
    (:NULL_ADMITTANCE_CONFIGURATION_CHANGED . 5)
    (:CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED . 6)
    (:JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED . 7)
    (:WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED . 8)
    (:WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED . 9)
    (:CONTROL_CONFIGURATION_FACTORY_RESTORED . 10)
    (:TOOL_CONFIGURATION_CHANGED . 11)
    (:PAYLOAD_CONFIGURATION_CHANGED . 12)
    (:CARTESIAN_REFERENCE_CHANGED . 13)
    (:CHANGE_CONTROL_MODE_FAILED . 14)
    (:JOINT_SPEED_SOFT_LIMITS_CHANGED . 16)
    (:TWIST_LINEAR_SOFT_LIMIT_CHANGED . 17)
    (:TWIST_ANGULAR_SOFT_LIMIT_CHANGED . 18)
    (:JOINT_ACCELERATION_SOFT_LIMITS_CHANGED . 19)
    (:DESIRED_TWIST_LINEAR_SPEED_CHANGED . 20)
    (:DESIRED_TWIST_ANGULAR_SPEED_CHANGED . 21)
    (:DESIRED_JOINT_SPEED_CHANGED . 22))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlConfigurationEvent)))
    "Constants for message type 'ControlConfigurationEvent"
  '((:UNSPECIFIED_CONTROL_CONFIGURATION_EVENT . 0)
    (:ANGLE_UNIT_CHANGED . 1)
    (:GRAVITY_VECTOR_CHANGED . 2)
    (:JOINT_ADMITTANCE_CONFIGURATION_CHANGED . 4)
    (:NULL_ADMITTANCE_CONFIGURATION_CHANGED . 5)
    (:CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED . 6)
    (:JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED . 7)
    (:WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED . 8)
    (:WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED . 9)
    (:CONTROL_CONFIGURATION_FACTORY_RESTORED . 10)
    (:TOOL_CONFIGURATION_CHANGED . 11)
    (:PAYLOAD_CONFIGURATION_CHANGED . 12)
    (:CARTESIAN_REFERENCE_CHANGED . 13)
    (:CHANGE_CONTROL_MODE_FAILED . 14)
    (:JOINT_SPEED_SOFT_LIMITS_CHANGED . 16)
    (:TWIST_LINEAR_SOFT_LIMIT_CHANGED . 17)
    (:TWIST_ANGULAR_SOFT_LIMIT_CHANGED . 18)
    (:JOINT_ACCELERATION_SOFT_LIMITS_CHANGED . 19)
    (:DESIRED_TWIST_LINEAR_SPEED_CHANGED . 20)
    (:DESIRED_TWIST_ANGULAR_SPEED_CHANGED . 21)
    (:DESIRED_JOINT_SPEED_CHANGED . 22))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfigurationEvent>) ostream)
  "Serializes a message object of type '<ControlConfigurationEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfigurationEvent>) istream)
  "Deserializes a message object of type '<ControlConfigurationEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfigurationEvent>)))
  "Returns string type for a message object of type '<ControlConfigurationEvent>"
  "kortex_driver/ControlConfigurationEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfigurationEvent)))
  "Returns string type for a message object of type 'ControlConfigurationEvent"
  "kortex_driver/ControlConfigurationEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfigurationEvent>)))
  "Returns md5sum for a message object of type '<ControlConfigurationEvent>"
  "5aa2a442ccb0c93e20aaef58c6149541")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfigurationEvent)))
  "Returns md5sum for a message object of type 'ControlConfigurationEvent"
  "5aa2a442ccb0c93e20aaef58c6149541")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfigurationEvent>)))
  "Returns full string definition for message of type '<ControlConfigurationEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROL_CONFIGURATION_EVENT = 0~%~%uint32 ANGLE_UNIT_CHANGED = 1~%~%uint32 GRAVITY_VECTOR_CHANGED = 2~%~%uint32 JOINT_ADMITTANCE_CONFIGURATION_CHANGED = 4~%~%uint32 NULL_ADMITTANCE_CONFIGURATION_CHANGED = 5~%~%uint32 CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED = 6~%~%uint32 JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED = 7~%~%uint32 WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED = 8~%~%uint32 WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED = 9~%~%uint32 CONTROL_CONFIGURATION_FACTORY_RESTORED = 10~%~%uint32 TOOL_CONFIGURATION_CHANGED = 11~%~%uint32 PAYLOAD_CONFIGURATION_CHANGED = 12~%~%uint32 CARTESIAN_REFERENCE_CHANGED = 13~%~%uint32 CHANGE_CONTROL_MODE_FAILED = 14~%~%uint32 JOINT_SPEED_SOFT_LIMITS_CHANGED = 16~%~%uint32 TWIST_LINEAR_SOFT_LIMIT_CHANGED = 17~%~%uint32 TWIST_ANGULAR_SOFT_LIMIT_CHANGED = 18~%~%uint32 JOINT_ACCELERATION_SOFT_LIMITS_CHANGED = 19~%~%uint32 DESIRED_TWIST_LINEAR_SPEED_CHANGED = 20~%~%uint32 DESIRED_TWIST_ANGULAR_SPEED_CHANGED = 21~%~%uint32 DESIRED_JOINT_SPEED_CHANGED = 22~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfigurationEvent)))
  "Returns full string definition for message of type 'ControlConfigurationEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROL_CONFIGURATION_EVENT = 0~%~%uint32 ANGLE_UNIT_CHANGED = 1~%~%uint32 GRAVITY_VECTOR_CHANGED = 2~%~%uint32 JOINT_ADMITTANCE_CONFIGURATION_CHANGED = 4~%~%uint32 NULL_ADMITTANCE_CONFIGURATION_CHANGED = 5~%~%uint32 CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED = 6~%~%uint32 JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED = 7~%~%uint32 WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED = 8~%~%uint32 WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED = 9~%~%uint32 CONTROL_CONFIGURATION_FACTORY_RESTORED = 10~%~%uint32 TOOL_CONFIGURATION_CHANGED = 11~%~%uint32 PAYLOAD_CONFIGURATION_CHANGED = 12~%~%uint32 CARTESIAN_REFERENCE_CHANGED = 13~%~%uint32 CHANGE_CONTROL_MODE_FAILED = 14~%~%uint32 JOINT_SPEED_SOFT_LIMITS_CHANGED = 16~%~%uint32 TWIST_LINEAR_SOFT_LIMIT_CHANGED = 17~%~%uint32 TWIST_ANGULAR_SOFT_LIMIT_CHANGED = 18~%~%uint32 JOINT_ACCELERATION_SOFT_LIMITS_CHANGED = 19~%~%uint32 DESIRED_TWIST_LINEAR_SPEED_CHANGED = 20~%~%uint32 DESIRED_TWIST_ANGULAR_SPEED_CHANGED = 21~%~%uint32 DESIRED_JOINT_SPEED_CHANGED = 22~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfigurationEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfigurationEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfigurationEvent
))
