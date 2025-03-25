; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlConfig_ControlMode.msg.html

(cl:defclass <ControlConfig_ControlMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControlConfig_ControlMode (<ControlConfig_ControlMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfig_ControlMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfig_ControlMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlConfig_ControlMode> is deprecated: use kortex_driver-msg:ControlConfig_ControlMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlConfig_ControlMode>)))
    "Constants for message type '<ControlConfig_ControlMode>"
  '((:UNSPECIFIED_CONTROL_MODE . 0)
    (:ANGULAR_JOYSTICK . 1)
    (:CARTESIAN_JOYSTICK . 2)
    (:ANGULAR_TRAJECTORY . 4)
    (:CARTESIAN_TRAJECTORY . 5)
    (:CARTESIAN_ADMITTANCE . 6)
    (:JOINT_ADMITTANCE . 7)
    (:NULL_SPACE_ADMITTANCE . 8)
    (:FORCE_CONTROL . 10)
    (:FORCE_CONTROL_MOTION_RESTRICTED . 11)
    (:CARTESIAN_WAYPOINT_TRAJECTORY . 12)
    (:IDLE . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlConfig_ControlMode)))
    "Constants for message type 'ControlConfig_ControlMode"
  '((:UNSPECIFIED_CONTROL_MODE . 0)
    (:ANGULAR_JOYSTICK . 1)
    (:CARTESIAN_JOYSTICK . 2)
    (:ANGULAR_TRAJECTORY . 4)
    (:CARTESIAN_TRAJECTORY . 5)
    (:CARTESIAN_ADMITTANCE . 6)
    (:JOINT_ADMITTANCE . 7)
    (:NULL_SPACE_ADMITTANCE . 8)
    (:FORCE_CONTROL . 10)
    (:FORCE_CONTROL_MOTION_RESTRICTED . 11)
    (:CARTESIAN_WAYPOINT_TRAJECTORY . 12)
    (:IDLE . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfig_ControlMode>) ostream)
  "Serializes a message object of type '<ControlConfig_ControlMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfig_ControlMode>) istream)
  "Deserializes a message object of type '<ControlConfig_ControlMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfig_ControlMode>)))
  "Returns string type for a message object of type '<ControlConfig_ControlMode>"
  "kortex_driver/ControlConfig_ControlMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_ControlMode)))
  "Returns string type for a message object of type 'ControlConfig_ControlMode"
  "kortex_driver/ControlConfig_ControlMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfig_ControlMode>)))
  "Returns md5sum for a message object of type '<ControlConfig_ControlMode>"
  "4a7e515c9084a41b0df1641cbb231ff3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfig_ControlMode)))
  "Returns md5sum for a message object of type 'ControlConfig_ControlMode"
  "4a7e515c9084a41b0df1641cbb231ff3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfig_ControlMode>)))
  "Returns full string definition for message of type '<ControlConfig_ControlMode>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROL_MODE = 0~%~%uint32 ANGULAR_JOYSTICK = 1~%~%uint32 CARTESIAN_JOYSTICK = 2~%~%uint32 ANGULAR_TRAJECTORY = 4~%~%uint32 CARTESIAN_TRAJECTORY = 5~%~%uint32 CARTESIAN_ADMITTANCE = 6~%~%uint32 JOINT_ADMITTANCE = 7~%~%uint32 NULL_SPACE_ADMITTANCE = 8~%~%uint32 FORCE_CONTROL = 10~%~%uint32 FORCE_CONTROL_MOTION_RESTRICTED = 11~%~%uint32 CARTESIAN_WAYPOINT_TRAJECTORY = 12~%~%uint32 IDLE = 13~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfig_ControlMode)))
  "Returns full string definition for message of type 'ControlConfig_ControlMode"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROL_MODE = 0~%~%uint32 ANGULAR_JOYSTICK = 1~%~%uint32 CARTESIAN_JOYSTICK = 2~%~%uint32 ANGULAR_TRAJECTORY = 4~%~%uint32 CARTESIAN_TRAJECTORY = 5~%~%uint32 CARTESIAN_ADMITTANCE = 6~%~%uint32 JOINT_ADMITTANCE = 7~%~%uint32 NULL_SPACE_ADMITTANCE = 8~%~%uint32 FORCE_CONTROL = 10~%~%uint32 FORCE_CONTROL_MOTION_RESTRICTED = 11~%~%uint32 CARTESIAN_WAYPOINT_TRAJECTORY = 12~%~%uint32 IDLE = 13~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfig_ControlMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfig_ControlMode>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfig_ControlMode
))
