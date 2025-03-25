; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude InterconnectCyclic_Command_tool_command.msg.html

(cl:defclass <InterconnectCyclic_Command_tool_command> (roslisp-msg-protocol:ros-message)
  ((gripper_command
    :reader gripper_command
    :initarg :gripper_command
    :type (cl:vector kortex_driver-msg:GripperCyclic_Command)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:GripperCyclic_Command :initial-element (cl:make-instance 'kortex_driver-msg:GripperCyclic_Command))))
)

(cl:defclass InterconnectCyclic_Command_tool_command (<InterconnectCyclic_Command_tool_command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterconnectCyclic_Command_tool_command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterconnectCyclic_Command_tool_command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<InterconnectCyclic_Command_tool_command> is deprecated: use kortex_driver-msg:InterconnectCyclic_Command_tool_command instead.")))

(cl:ensure-generic-function 'gripper_command-val :lambda-list '(m))
(cl:defmethod gripper_command-val ((m <InterconnectCyclic_Command_tool_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:gripper_command-val is deprecated.  Use kortex_driver-msg:gripper_command instead.")
  (gripper_command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterconnectCyclic_Command_tool_command>) ostream)
  "Serializes a message object of type '<InterconnectCyclic_Command_tool_command>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gripper_command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gripper_command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterconnectCyclic_Command_tool_command>) istream)
  "Deserializes a message object of type '<InterconnectCyclic_Command_tool_command>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gripper_command) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gripper_command)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:GripperCyclic_Command))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterconnectCyclic_Command_tool_command>)))
  "Returns string type for a message object of type '<InterconnectCyclic_Command_tool_command>"
  "kortex_driver/InterconnectCyclic_Command_tool_command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterconnectCyclic_Command_tool_command)))
  "Returns string type for a message object of type 'InterconnectCyclic_Command_tool_command"
  "kortex_driver/InterconnectCyclic_Command_tool_command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterconnectCyclic_Command_tool_command>)))
  "Returns md5sum for a message object of type '<InterconnectCyclic_Command_tool_command>"
  "cb4b9ede5a008a07d439d113030c34d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterconnectCyclic_Command_tool_command)))
  "Returns md5sum for a message object of type 'InterconnectCyclic_Command_tool_command"
  "cb4b9ede5a008a07d439d113030c34d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterconnectCyclic_Command_tool_command>)))
  "Returns full string definition for message of type '<InterconnectCyclic_Command_tool_command>"
  (cl:format cl:nil "~%GripperCyclic_Command[] gripper_command~%================================================================================~%MSG: kortex_driver/GripperCyclic_Command~%~%GripperCyclic_MessageId command_id~%uint32 flags~%MotorCommand[] motor_cmd~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorCommand~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 force~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterconnectCyclic_Command_tool_command)))
  "Returns full string definition for message of type 'InterconnectCyclic_Command_tool_command"
  (cl:format cl:nil "~%GripperCyclic_Command[] gripper_command~%================================================================================~%MSG: kortex_driver/GripperCyclic_Command~%~%GripperCyclic_MessageId command_id~%uint32 flags~%MotorCommand[] motor_cmd~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorCommand~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 force~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterconnectCyclic_Command_tool_command>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gripper_command) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterconnectCyclic_Command_tool_command>))
  "Converts a ROS message object to a list"
  (cl:list 'InterconnectCyclic_Command_tool_command
    (cl:cons ':gripper_command (gripper_command msg))
))
