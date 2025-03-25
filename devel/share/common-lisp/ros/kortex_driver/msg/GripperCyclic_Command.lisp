; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GripperCyclic_Command.msg.html

(cl:defclass <GripperCyclic_Command> (roslisp-msg-protocol:ros-message)
  ((command_id
    :reader command_id
    :initarg :command_id
    :type kortex_driver-msg:GripperCyclic_MessageId
    :initform (cl:make-instance 'kortex_driver-msg:GripperCyclic_MessageId))
   (flags
    :reader flags
    :initarg :flags
    :type cl:integer
    :initform 0)
   (motor_cmd
    :reader motor_cmd
    :initarg :motor_cmd
    :type (cl:vector kortex_driver-msg:MotorCommand)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:MotorCommand :initial-element (cl:make-instance 'kortex_driver-msg:MotorCommand))))
)

(cl:defclass GripperCyclic_Command (<GripperCyclic_Command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperCyclic_Command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperCyclic_Command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GripperCyclic_Command> is deprecated: use kortex_driver-msg:GripperCyclic_Command instead.")))

(cl:ensure-generic-function 'command_id-val :lambda-list '(m))
(cl:defmethod command_id-val ((m <GripperCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:command_id-val is deprecated.  Use kortex_driver-msg:command_id instead.")
  (command_id m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <GripperCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:flags-val is deprecated.  Use kortex_driver-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'motor_cmd-val :lambda-list '(m))
(cl:defmethod motor_cmd-val ((m <GripperCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:motor_cmd-val is deprecated.  Use kortex_driver-msg:motor_cmd instead.")
  (motor_cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperCyclic_Command>) ostream)
  "Serializes a message object of type '<GripperCyclic_Command>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command_id) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motor_cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperCyclic_Command>) istream)
  "Deserializes a message object of type '<GripperCyclic_Command>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command_id) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_cmd) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor_cmd)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:MotorCommand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperCyclic_Command>)))
  "Returns string type for a message object of type '<GripperCyclic_Command>"
  "kortex_driver/GripperCyclic_Command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperCyclic_Command)))
  "Returns string type for a message object of type 'GripperCyclic_Command"
  "kortex_driver/GripperCyclic_Command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperCyclic_Command>)))
  "Returns md5sum for a message object of type '<GripperCyclic_Command>"
  "aa88200f31c06368a2a9758c0985b419")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperCyclic_Command)))
  "Returns md5sum for a message object of type 'GripperCyclic_Command"
  "aa88200f31c06368a2a9758c0985b419")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperCyclic_Command>)))
  "Returns full string definition for message of type '<GripperCyclic_Command>"
  (cl:format cl:nil "~%GripperCyclic_MessageId command_id~%uint32 flags~%MotorCommand[] motor_cmd~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorCommand~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 force~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperCyclic_Command)))
  "Returns full string definition for message of type 'GripperCyclic_Command"
  (cl:format cl:nil "~%GripperCyclic_MessageId command_id~%uint32 flags~%MotorCommand[] motor_cmd~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorCommand~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 force~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperCyclic_Command>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command_id))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperCyclic_Command>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperCyclic_Command
    (cl:cons ':command_id (command_id msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':motor_cmd (motor_cmd msg))
))
