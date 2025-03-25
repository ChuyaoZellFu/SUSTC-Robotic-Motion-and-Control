; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude InterconnectCyclic_Command.msg.html

(cl:defclass <InterconnectCyclic_Command> (roslisp-msg-protocol:ros-message)
  ((command_id
    :reader command_id
    :initarg :command_id
    :type kortex_driver-msg:InterconnectCyclic_MessageId
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectCyclic_MessageId))
   (flags
    :reader flags
    :initarg :flags
    :type cl:integer
    :initform 0)
   (oneof_tool_command
    :reader oneof_tool_command
    :initarg :oneof_tool_command
    :type kortex_driver-msg:InterconnectCyclic_Command_tool_command
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectCyclic_Command_tool_command)))
)

(cl:defclass InterconnectCyclic_Command (<InterconnectCyclic_Command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterconnectCyclic_Command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterconnectCyclic_Command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<InterconnectCyclic_Command> is deprecated: use kortex_driver-msg:InterconnectCyclic_Command instead.")))

(cl:ensure-generic-function 'command_id-val :lambda-list '(m))
(cl:defmethod command_id-val ((m <InterconnectCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:command_id-val is deprecated.  Use kortex_driver-msg:command_id instead.")
  (command_id m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <InterconnectCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:flags-val is deprecated.  Use kortex_driver-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'oneof_tool_command-val :lambda-list '(m))
(cl:defmethod oneof_tool_command-val ((m <InterconnectCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_tool_command-val is deprecated.  Use kortex_driver-msg:oneof_tool_command instead.")
  (oneof_tool_command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterconnectCyclic_Command>) ostream)
  "Serializes a message object of type '<InterconnectCyclic_Command>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command_id) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_tool_command) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterconnectCyclic_Command>) istream)
  "Deserializes a message object of type '<InterconnectCyclic_Command>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command_id) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_tool_command) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterconnectCyclic_Command>)))
  "Returns string type for a message object of type '<InterconnectCyclic_Command>"
  "kortex_driver/InterconnectCyclic_Command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterconnectCyclic_Command)))
  "Returns string type for a message object of type 'InterconnectCyclic_Command"
  "kortex_driver/InterconnectCyclic_Command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterconnectCyclic_Command>)))
  "Returns md5sum for a message object of type '<InterconnectCyclic_Command>"
  "4def369ad2d800b446113efeb710b701")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterconnectCyclic_Command)))
  "Returns md5sum for a message object of type 'InterconnectCyclic_Command"
  "4def369ad2d800b446113efeb710b701")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterconnectCyclic_Command>)))
  "Returns full string definition for message of type '<InterconnectCyclic_Command>"
  (cl:format cl:nil "~%InterconnectCyclic_MessageId command_id~%uint32 flags~%InterconnectCyclic_Command_tool_command oneof_tool_command~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Command_tool_command~%~%GripperCyclic_Command[] gripper_command~%================================================================================~%MSG: kortex_driver/GripperCyclic_Command~%~%GripperCyclic_MessageId command_id~%uint32 flags~%MotorCommand[] motor_cmd~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorCommand~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 force~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterconnectCyclic_Command)))
  "Returns full string definition for message of type 'InterconnectCyclic_Command"
  (cl:format cl:nil "~%InterconnectCyclic_MessageId command_id~%uint32 flags~%InterconnectCyclic_Command_tool_command oneof_tool_command~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Command_tool_command~%~%GripperCyclic_Command[] gripper_command~%================================================================================~%MSG: kortex_driver/GripperCyclic_Command~%~%GripperCyclic_MessageId command_id~%uint32 flags~%MotorCommand[] motor_cmd~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorCommand~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 force~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterconnectCyclic_Command>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command_id))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_tool_command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterconnectCyclic_Command>))
  "Converts a ROS message object to a list"
  (cl:list 'InterconnectCyclic_Command
    (cl:cons ':command_id (command_id msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':oneof_tool_command (oneof_tool_command msg))
))
