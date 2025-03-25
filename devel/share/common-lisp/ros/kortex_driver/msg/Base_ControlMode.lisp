; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Base_ControlMode.msg.html

(cl:defclass <Base_ControlMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Base_ControlMode (<Base_ControlMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_ControlMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_ControlMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Base_ControlMode> is deprecated: use kortex_driver-msg:Base_ControlMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Base_ControlMode>)))
    "Constants for message type '<Base_ControlMode>"
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
    (:IDLE . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Base_ControlMode)))
    "Constants for message type 'Base_ControlMode"
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
    (:IDLE . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_ControlMode>) ostream)
  "Serializes a message object of type '<Base_ControlMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_ControlMode>) istream)
  "Deserializes a message object of type '<Base_ControlMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_ControlMode>)))
  "Returns string type for a message object of type '<Base_ControlMode>"
  "kortex_driver/Base_ControlMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_ControlMode)))
  "Returns string type for a message object of type 'Base_ControlMode"
  "kortex_driver/Base_ControlMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_ControlMode>)))
  "Returns md5sum for a message object of type '<Base_ControlMode>"
  "027ce394a2c0dcd8cf27153deda7b144")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_ControlMode)))
  "Returns md5sum for a message object of type 'Base_ControlMode"
  "027ce394a2c0dcd8cf27153deda7b144")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_ControlMode>)))
  "Returns full string definition for message of type '<Base_ControlMode>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROL_MODE = 0~%~%uint32 ANGULAR_JOYSTICK = 1~%~%uint32 CARTESIAN_JOYSTICK = 2~%~%uint32 ANGULAR_TRAJECTORY = 4~%~%uint32 CARTESIAN_TRAJECTORY = 5~%~%uint32 CARTESIAN_ADMITTANCE = 6~%~%uint32 JOINT_ADMITTANCE = 7~%~%uint32 NULL_SPACE_ADMITTANCE = 8~%~%uint32 FORCE_CONTROL = 10~%~%uint32 FORCE_CONTROL_MOTION_RESTRICTED = 11~%~%uint32 IDLE = 13~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_ControlMode)))
  "Returns full string definition for message of type 'Base_ControlMode"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROL_MODE = 0~%~%uint32 ANGULAR_JOYSTICK = 1~%~%uint32 CARTESIAN_JOYSTICK = 2~%~%uint32 ANGULAR_TRAJECTORY = 4~%~%uint32 CARTESIAN_TRAJECTORY = 5~%~%uint32 CARTESIAN_ADMITTANCE = 6~%~%uint32 JOINT_ADMITTANCE = 7~%~%uint32 NULL_SPACE_ADMITTANCE = 8~%~%uint32 FORCE_CONTROL = 10~%~%uint32 FORCE_CONTROL_MOTION_RESTRICTED = 11~%~%uint32 IDLE = 13~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_ControlMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_ControlMode>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_ControlMode
))
