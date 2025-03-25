; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActuatorConfig_ControlMode.msg.html

(cl:defclass <ActuatorConfig_ControlMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ActuatorConfig_ControlMode (<ActuatorConfig_ControlMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorConfig_ControlMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorConfig_ControlMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActuatorConfig_ControlMode> is deprecated: use kortex_driver-msg:ActuatorConfig_ControlMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ActuatorConfig_ControlMode>)))
    "Constants for message type '<ActuatorConfig_ControlMode>"
  '((:NONE . 0)
    (:POSITION . 1)
    (:VELOCITY . 2)
    (:TORQUE . 3)
    (:CURRENT . 4)
    (:CUSTOM . 5)
    (:TORQUE_HIGH_VELOCITY . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ActuatorConfig_ControlMode)))
    "Constants for message type 'ActuatorConfig_ControlMode"
  '((:NONE . 0)
    (:POSITION . 1)
    (:VELOCITY . 2)
    (:TORQUE . 3)
    (:CURRENT . 4)
    (:CUSTOM . 5)
    (:TORQUE_HIGH_VELOCITY . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorConfig_ControlMode>) ostream)
  "Serializes a message object of type '<ActuatorConfig_ControlMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorConfig_ControlMode>) istream)
  "Deserializes a message object of type '<ActuatorConfig_ControlMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorConfig_ControlMode>)))
  "Returns string type for a message object of type '<ActuatorConfig_ControlMode>"
  "kortex_driver/ActuatorConfig_ControlMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorConfig_ControlMode)))
  "Returns string type for a message object of type 'ActuatorConfig_ControlMode"
  "kortex_driver/ActuatorConfig_ControlMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorConfig_ControlMode>)))
  "Returns md5sum for a message object of type '<ActuatorConfig_ControlMode>"
  "bb12ce17d437184d7baad58c2fe8b556")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorConfig_ControlMode)))
  "Returns md5sum for a message object of type 'ActuatorConfig_ControlMode"
  "bb12ce17d437184d7baad58c2fe8b556")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorConfig_ControlMode>)))
  "Returns full string definition for message of type '<ActuatorConfig_ControlMode>"
  (cl:format cl:nil "~%uint32 NONE = 0~%~%uint32 POSITION = 1~%~%uint32 VELOCITY = 2~%~%uint32 TORQUE = 3~%~%uint32 CURRENT = 4~%~%uint32 CUSTOM = 5~%~%uint32 TORQUE_HIGH_VELOCITY = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorConfig_ControlMode)))
  "Returns full string definition for message of type 'ActuatorConfig_ControlMode"
  (cl:format cl:nil "~%uint32 NONE = 0~%~%uint32 POSITION = 1~%~%uint32 VELOCITY = 2~%~%uint32 TORQUE = 3~%~%uint32 CURRENT = 4~%~%uint32 CUSTOM = 5~%~%uint32 TORQUE_HIGH_VELOCITY = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorConfig_ControlMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorConfig_ControlMode>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorConfig_ControlMode
))
