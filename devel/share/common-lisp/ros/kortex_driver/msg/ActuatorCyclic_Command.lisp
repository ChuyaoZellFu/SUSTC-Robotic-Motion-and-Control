; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActuatorCyclic_Command.msg.html

(cl:defclass <ActuatorCyclic_Command> (roslisp-msg-protocol:ros-message)
  ((command_id
    :reader command_id
    :initarg :command_id
    :type kortex_driver-msg:ActuatorCyclic_MessageId
    :initform (cl:make-instance 'kortex_driver-msg:ActuatorCyclic_MessageId))
   (flags
    :reader flags
    :initarg :flags
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (torque_joint
    :reader torque_joint
    :initarg :torque_joint
    :type cl:float
    :initform 0.0)
   (current_motor
    :reader current_motor
    :initarg :current_motor
    :type cl:float
    :initform 0.0))
)

(cl:defclass ActuatorCyclic_Command (<ActuatorCyclic_Command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorCyclic_Command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorCyclic_Command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActuatorCyclic_Command> is deprecated: use kortex_driver-msg:ActuatorCyclic_Command instead.")))

(cl:ensure-generic-function 'command_id-val :lambda-list '(m))
(cl:defmethod command_id-val ((m <ActuatorCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:command_id-val is deprecated.  Use kortex_driver-msg:command_id instead.")
  (command_id m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <ActuatorCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:flags-val is deprecated.  Use kortex_driver-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ActuatorCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:position-val is deprecated.  Use kortex_driver-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <ActuatorCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:velocity-val is deprecated.  Use kortex_driver-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'torque_joint-val :lambda-list '(m))
(cl:defmethod torque_joint-val ((m <ActuatorCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:torque_joint-val is deprecated.  Use kortex_driver-msg:torque_joint instead.")
  (torque_joint m))

(cl:ensure-generic-function 'current_motor-val :lambda-list '(m))
(cl:defmethod current_motor-val ((m <ActuatorCyclic_Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:current_motor-val is deprecated.  Use kortex_driver-msg:current_motor instead.")
  (current_motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorCyclic_Command>) ostream)
  "Serializes a message object of type '<ActuatorCyclic_Command>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command_id) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorCyclic_Command>) istream)
  "Deserializes a message object of type '<ActuatorCyclic_Command>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command_id) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_joint) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_motor) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorCyclic_Command>)))
  "Returns string type for a message object of type '<ActuatorCyclic_Command>"
  "kortex_driver/ActuatorCyclic_Command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorCyclic_Command)))
  "Returns string type for a message object of type 'ActuatorCyclic_Command"
  "kortex_driver/ActuatorCyclic_Command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorCyclic_Command>)))
  "Returns md5sum for a message object of type '<ActuatorCyclic_Command>"
  "c79b9ac4bc4567cb00f9d7a7a89a1be9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorCyclic_Command)))
  "Returns md5sum for a message object of type 'ActuatorCyclic_Command"
  "c79b9ac4bc4567cb00f9d7a7a89a1be9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorCyclic_Command>)))
  "Returns full string definition for message of type '<ActuatorCyclic_Command>"
  (cl:format cl:nil "~%ActuatorCyclic_MessageId command_id~%uint32 flags~%float32 position~%float32 velocity~%float32 torque_joint~%float32 current_motor~%================================================================================~%MSG: kortex_driver/ActuatorCyclic_MessageId~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorCyclic_Command)))
  "Returns full string definition for message of type 'ActuatorCyclic_Command"
  (cl:format cl:nil "~%ActuatorCyclic_MessageId command_id~%uint32 flags~%float32 position~%float32 velocity~%float32 torque_joint~%float32 current_motor~%================================================================================~%MSG: kortex_driver/ActuatorCyclic_MessageId~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorCyclic_Command>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command_id))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorCyclic_Command>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorCyclic_Command
    (cl:cons ':command_id (command_id msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':torque_joint (torque_joint msg))
    (cl:cons ':current_motor (current_motor msg))
))
