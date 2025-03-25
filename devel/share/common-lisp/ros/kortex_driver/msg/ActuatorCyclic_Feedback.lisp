; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActuatorCyclic_Feedback.msg.html

(cl:defclass <ActuatorCyclic_Feedback> (roslisp-msg-protocol:ros-message)
  ((feedback_id
    :reader feedback_id
    :initarg :feedback_id
    :type kortex_driver-msg:ActuatorCyclic_MessageId
    :initform (cl:make-instance 'kortex_driver-msg:ActuatorCyclic_MessageId))
   (status_flags
    :reader status_flags
    :initarg :status_flags
    :type cl:integer
    :initform 0)
   (jitter_comm
    :reader jitter_comm
    :initarg :jitter_comm
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
   (torque
    :reader torque
    :initarg :torque
    :type cl:float
    :initform 0.0)
   (current_motor
    :reader current_motor
    :initarg :current_motor
    :type cl:float
    :initform 0.0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (temperature_motor
    :reader temperature_motor
    :initarg :temperature_motor
    :type cl:float
    :initform 0.0)
   (temperature_core
    :reader temperature_core
    :initarg :temperature_core
    :type cl:float
    :initform 0.0)
   (fault_bank_a
    :reader fault_bank_a
    :initarg :fault_bank_a
    :type cl:integer
    :initform 0)
   (fault_bank_b
    :reader fault_bank_b
    :initarg :fault_bank_b
    :type cl:integer
    :initform 0)
   (warning_bank_a
    :reader warning_bank_a
    :initarg :warning_bank_a
    :type cl:integer
    :initform 0)
   (warning_bank_b
    :reader warning_bank_b
    :initarg :warning_bank_b
    :type cl:integer
    :initform 0))
)

(cl:defclass ActuatorCyclic_Feedback (<ActuatorCyclic_Feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorCyclic_Feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorCyclic_Feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActuatorCyclic_Feedback> is deprecated: use kortex_driver-msg:ActuatorCyclic_Feedback instead.")))

(cl:ensure-generic-function 'feedback_id-val :lambda-list '(m))
(cl:defmethod feedback_id-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:feedback_id-val is deprecated.  Use kortex_driver-msg:feedback_id instead.")
  (feedback_id m))

(cl:ensure-generic-function 'status_flags-val :lambda-list '(m))
(cl:defmethod status_flags-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:status_flags-val is deprecated.  Use kortex_driver-msg:status_flags instead.")
  (status_flags m))

(cl:ensure-generic-function 'jitter_comm-val :lambda-list '(m))
(cl:defmethod jitter_comm-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:jitter_comm-val is deprecated.  Use kortex_driver-msg:jitter_comm instead.")
  (jitter_comm m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:position-val is deprecated.  Use kortex_driver-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:velocity-val is deprecated.  Use kortex_driver-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'torque-val :lambda-list '(m))
(cl:defmethod torque-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:torque-val is deprecated.  Use kortex_driver-msg:torque instead.")
  (torque m))

(cl:ensure-generic-function 'current_motor-val :lambda-list '(m))
(cl:defmethod current_motor-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:current_motor-val is deprecated.  Use kortex_driver-msg:current_motor instead.")
  (current_motor m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:voltage-val is deprecated.  Use kortex_driver-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'temperature_motor-val :lambda-list '(m))
(cl:defmethod temperature_motor-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:temperature_motor-val is deprecated.  Use kortex_driver-msg:temperature_motor instead.")
  (temperature_motor m))

(cl:ensure-generic-function 'temperature_core-val :lambda-list '(m))
(cl:defmethod temperature_core-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:temperature_core-val is deprecated.  Use kortex_driver-msg:temperature_core instead.")
  (temperature_core m))

(cl:ensure-generic-function 'fault_bank_a-val :lambda-list '(m))
(cl:defmethod fault_bank_a-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:fault_bank_a-val is deprecated.  Use kortex_driver-msg:fault_bank_a instead.")
  (fault_bank_a m))

(cl:ensure-generic-function 'fault_bank_b-val :lambda-list '(m))
(cl:defmethod fault_bank_b-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:fault_bank_b-val is deprecated.  Use kortex_driver-msg:fault_bank_b instead.")
  (fault_bank_b m))

(cl:ensure-generic-function 'warning_bank_a-val :lambda-list '(m))
(cl:defmethod warning_bank_a-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_bank_a-val is deprecated.  Use kortex_driver-msg:warning_bank_a instead.")
  (warning_bank_a m))

(cl:ensure-generic-function 'warning_bank_b-val :lambda-list '(m))
(cl:defmethod warning_bank_b-val ((m <ActuatorCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_bank_b-val is deprecated.  Use kortex_driver-msg:warning_bank_b instead.")
  (warning_bank_b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorCyclic_Feedback>) ostream)
  "Serializes a message object of type '<ActuatorCyclic_Feedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feedback_id) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'jitter_comm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'jitter_comm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'jitter_comm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'jitter_comm)) ostream)
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature_motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature_core))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'warning_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'warning_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'warning_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'warning_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'warning_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'warning_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'warning_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'warning_bank_b)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorCyclic_Feedback>) istream)
  "Deserializes a message object of type '<ActuatorCyclic_Feedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'feedback_id) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'jitter_comm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'jitter_comm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'jitter_comm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'jitter_comm)) (cl:read-byte istream))
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
    (cl:setf (cl:slot-value msg 'torque) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_motor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature_motor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature_core) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'warning_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'warning_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'warning_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'warning_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'warning_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'warning_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'warning_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'warning_bank_b)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorCyclic_Feedback>)))
  "Returns string type for a message object of type '<ActuatorCyclic_Feedback>"
  "kortex_driver/ActuatorCyclic_Feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorCyclic_Feedback)))
  "Returns string type for a message object of type 'ActuatorCyclic_Feedback"
  "kortex_driver/ActuatorCyclic_Feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorCyclic_Feedback>)))
  "Returns md5sum for a message object of type '<ActuatorCyclic_Feedback>"
  "299d9bdfeb28700b38cf7f19f730d6c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorCyclic_Feedback)))
  "Returns md5sum for a message object of type 'ActuatorCyclic_Feedback"
  "299d9bdfeb28700b38cf7f19f730d6c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorCyclic_Feedback>)))
  "Returns full string definition for message of type '<ActuatorCyclic_Feedback>"
  (cl:format cl:nil "~%ActuatorCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 jitter_comm~%float32 position~%float32 velocity~%float32 torque~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%float32 temperature_core~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%================================================================================~%MSG: kortex_driver/ActuatorCyclic_MessageId~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorCyclic_Feedback)))
  "Returns full string definition for message of type 'ActuatorCyclic_Feedback"
  (cl:format cl:nil "~%ActuatorCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 jitter_comm~%float32 position~%float32 velocity~%float32 torque~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%float32 temperature_core~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%================================================================================~%MSG: kortex_driver/ActuatorCyclic_MessageId~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorCyclic_Feedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'feedback_id))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorCyclic_Feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorCyclic_Feedback
    (cl:cons ':feedback_id (feedback_id msg))
    (cl:cons ':status_flags (status_flags msg))
    (cl:cons ':jitter_comm (jitter_comm msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':torque (torque msg))
    (cl:cons ':current_motor (current_motor msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':temperature_motor (temperature_motor msg))
    (cl:cons ':temperature_core (temperature_core msg))
    (cl:cons ':fault_bank_a (fault_bank_a msg))
    (cl:cons ':fault_bank_b (fault_bank_b msg))
    (cl:cons ':warning_bank_a (warning_bank_a msg))
    (cl:cons ':warning_bank_b (warning_bank_b msg))
))
