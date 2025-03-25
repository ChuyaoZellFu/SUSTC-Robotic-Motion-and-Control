; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BaseCyclic_Command.msg.html

(cl:defclass <BaseCyclic_Command> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:integer
    :initform 0)
   (actuators
    :reader actuators
    :initarg :actuators
    :type (cl:vector kortex_driver-msg:ActuatorCommand)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ActuatorCommand :initial-element (cl:make-instance 'kortex_driver-msg:ActuatorCommand)))
   (interconnect
    :reader interconnect
    :initarg :interconnect
    :type kortex_driver-msg:InterconnectCyclic_Command
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectCyclic_Command)))
)

(cl:defclass BaseCyclic_Command (<BaseCyclic_Command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseCyclic_Command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseCyclic_Command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BaseCyclic_Command> is deprecated: use kortex_driver-msg:BaseCyclic_Command instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <BaseCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:frame_id-val is deprecated.  Use kortex_driver-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'actuators-val :lambda-list '(m))
(cl:defmethod actuators-val ((m <BaseCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:actuators-val is deprecated.  Use kortex_driver-msg:actuators instead.")
  (actuators m))

(cl:ensure-generic-function 'interconnect-val :lambda-list '(m))
(cl:defmethod interconnect-val ((m <BaseCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:interconnect-val is deprecated.  Use kortex_driver-msg:interconnect instead.")
  (interconnect m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseCyclic_Command>) ostream)
  "Serializes a message object of type '<BaseCyclic_Command>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'actuators))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'actuators))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'interconnect) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseCyclic_Command>) istream)
  "Deserializes a message object of type '<BaseCyclic_Command>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'actuators) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'actuators)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ActuatorCommand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'interconnect) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseCyclic_Command>)))
  "Returns string type for a message object of type '<BaseCyclic_Command>"
  "kortex_driver/BaseCyclic_Command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseCyclic_Command)))
  "Returns string type for a message object of type 'BaseCyclic_Command"
  "kortex_driver/BaseCyclic_Command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseCyclic_Command>)))
  "Returns md5sum for a message object of type '<BaseCyclic_Command>"
  "79e9d628025fc54837a4192c98fc5b9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseCyclic_Command)))
  "Returns md5sum for a message object of type 'BaseCyclic_Command"
  "79e9d628025fc54837a4192c98fc5b9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseCyclic_Command>)))
  "Returns full string definition for message of type '<BaseCyclic_Command>"
  (cl:format cl:nil "~%uint32 frame_id~%ActuatorCommand[] actuators~%InterconnectCyclic_Command interconnect~%================================================================================~%MSG: kortex_driver/ActuatorCommand~%~%uint32 command_id~%uint32 flags~%float32 position~%float32 velocity~%float32 torque_joint~%float32 current_motor~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Command~%~%InterconnectCyclic_MessageId command_id~%uint32 flags~%InterconnectCyclic_Command_tool_command oneof_tool_command~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Command_tool_command~%~%GripperCyclic_Command[] gripper_command~%================================================================================~%MSG: kortex_driver/GripperCyclic_Command~%~%GripperCyclic_MessageId command_id~%uint32 flags~%MotorCommand[] motor_cmd~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorCommand~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 force~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseCyclic_Command)))
  "Returns full string definition for message of type 'BaseCyclic_Command"
  (cl:format cl:nil "~%uint32 frame_id~%ActuatorCommand[] actuators~%InterconnectCyclic_Command interconnect~%================================================================================~%MSG: kortex_driver/ActuatorCommand~%~%uint32 command_id~%uint32 flags~%float32 position~%float32 velocity~%float32 torque_joint~%float32 current_motor~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Command~%~%InterconnectCyclic_MessageId command_id~%uint32 flags~%InterconnectCyclic_Command_tool_command oneof_tool_command~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Command_tool_command~%~%GripperCyclic_Command[] gripper_command~%================================================================================~%MSG: kortex_driver/GripperCyclic_Command~%~%GripperCyclic_MessageId command_id~%uint32 flags~%MotorCommand[] motor_cmd~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorCommand~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 force~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseCyclic_Command>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'actuators) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'interconnect))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseCyclic_Command>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseCyclic_Command
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':actuators (actuators msg))
    (cl:cons ':interconnect (interconnect msg))
))
