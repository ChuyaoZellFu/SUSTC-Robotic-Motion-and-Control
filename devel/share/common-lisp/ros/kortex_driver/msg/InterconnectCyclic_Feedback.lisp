; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude InterconnectCyclic_Feedback.msg.html

(cl:defclass <InterconnectCyclic_Feedback> (roslisp-msg-protocol:ros-message)
  ((feedback_id
    :reader feedback_id
    :initarg :feedback_id
    :type kortex_driver-msg:InterconnectCyclic_MessageId
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectCyclic_MessageId))
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
   (imu_acceleration_x
    :reader imu_acceleration_x
    :initarg :imu_acceleration_x
    :type cl:float
    :initform 0.0)
   (imu_acceleration_y
    :reader imu_acceleration_y
    :initarg :imu_acceleration_y
    :type cl:float
    :initform 0.0)
   (imu_acceleration_z
    :reader imu_acceleration_z
    :initarg :imu_acceleration_z
    :type cl:float
    :initform 0.0)
   (imu_angular_velocity_x
    :reader imu_angular_velocity_x
    :initarg :imu_angular_velocity_x
    :type cl:float
    :initform 0.0)
   (imu_angular_velocity_y
    :reader imu_angular_velocity_y
    :initarg :imu_angular_velocity_y
    :type cl:float
    :initform 0.0)
   (imu_angular_velocity_z
    :reader imu_angular_velocity_z
    :initarg :imu_angular_velocity_z
    :type cl:float
    :initform 0.0)
   (voltage
    :reader voltage
    :initarg :voltage
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
    :initform 0)
   (oneof_tool_feedback
    :reader oneof_tool_feedback
    :initarg :oneof_tool_feedback
    :type kortex_driver-msg:InterconnectCyclic_Feedback_tool_feedback
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectCyclic_Feedback_tool_feedback)))
)

(cl:defclass InterconnectCyclic_Feedback (<InterconnectCyclic_Feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterconnectCyclic_Feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterconnectCyclic_Feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<InterconnectCyclic_Feedback> is deprecated: use kortex_driver-msg:InterconnectCyclic_Feedback instead.")))

(cl:ensure-generic-function 'feedback_id-val :lambda-list '(m))
(cl:defmethod feedback_id-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:feedback_id-val is deprecated.  Use kortex_driver-msg:feedback_id instead.")
  (feedback_id m))

(cl:ensure-generic-function 'status_flags-val :lambda-list '(m))
(cl:defmethod status_flags-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:status_flags-val is deprecated.  Use kortex_driver-msg:status_flags instead.")
  (status_flags m))

(cl:ensure-generic-function 'jitter_comm-val :lambda-list '(m))
(cl:defmethod jitter_comm-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:jitter_comm-val is deprecated.  Use kortex_driver-msg:jitter_comm instead.")
  (jitter_comm m))

(cl:ensure-generic-function 'imu_acceleration_x-val :lambda-list '(m))
(cl:defmethod imu_acceleration_x-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_acceleration_x-val is deprecated.  Use kortex_driver-msg:imu_acceleration_x instead.")
  (imu_acceleration_x m))

(cl:ensure-generic-function 'imu_acceleration_y-val :lambda-list '(m))
(cl:defmethod imu_acceleration_y-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_acceleration_y-val is deprecated.  Use kortex_driver-msg:imu_acceleration_y instead.")
  (imu_acceleration_y m))

(cl:ensure-generic-function 'imu_acceleration_z-val :lambda-list '(m))
(cl:defmethod imu_acceleration_z-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_acceleration_z-val is deprecated.  Use kortex_driver-msg:imu_acceleration_z instead.")
  (imu_acceleration_z m))

(cl:ensure-generic-function 'imu_angular_velocity_x-val :lambda-list '(m))
(cl:defmethod imu_angular_velocity_x-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_angular_velocity_x-val is deprecated.  Use kortex_driver-msg:imu_angular_velocity_x instead.")
  (imu_angular_velocity_x m))

(cl:ensure-generic-function 'imu_angular_velocity_y-val :lambda-list '(m))
(cl:defmethod imu_angular_velocity_y-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_angular_velocity_y-val is deprecated.  Use kortex_driver-msg:imu_angular_velocity_y instead.")
  (imu_angular_velocity_y m))

(cl:ensure-generic-function 'imu_angular_velocity_z-val :lambda-list '(m))
(cl:defmethod imu_angular_velocity_z-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_angular_velocity_z-val is deprecated.  Use kortex_driver-msg:imu_angular_velocity_z instead.")
  (imu_angular_velocity_z m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:voltage-val is deprecated.  Use kortex_driver-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'temperature_core-val :lambda-list '(m))
(cl:defmethod temperature_core-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:temperature_core-val is deprecated.  Use kortex_driver-msg:temperature_core instead.")
  (temperature_core m))

(cl:ensure-generic-function 'fault_bank_a-val :lambda-list '(m))
(cl:defmethod fault_bank_a-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:fault_bank_a-val is deprecated.  Use kortex_driver-msg:fault_bank_a instead.")
  (fault_bank_a m))

(cl:ensure-generic-function 'fault_bank_b-val :lambda-list '(m))
(cl:defmethod fault_bank_b-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:fault_bank_b-val is deprecated.  Use kortex_driver-msg:fault_bank_b instead.")
  (fault_bank_b m))

(cl:ensure-generic-function 'warning_bank_a-val :lambda-list '(m))
(cl:defmethod warning_bank_a-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_bank_a-val is deprecated.  Use kortex_driver-msg:warning_bank_a instead.")
  (warning_bank_a m))

(cl:ensure-generic-function 'warning_bank_b-val :lambda-list '(m))
(cl:defmethod warning_bank_b-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_bank_b-val is deprecated.  Use kortex_driver-msg:warning_bank_b instead.")
  (warning_bank_b m))

(cl:ensure-generic-function 'oneof_tool_feedback-val :lambda-list '(m))
(cl:defmethod oneof_tool_feedback-val ((m <InterconnectCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_tool_feedback-val is deprecated.  Use kortex_driver-msg:oneof_tool_feedback instead.")
  (oneof_tool_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterconnectCyclic_Feedback>) ostream)
  "Serializes a message object of type '<InterconnectCyclic_Feedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feedback_id) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'jitter_comm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'jitter_comm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'jitter_comm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'jitter_comm)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_acceleration_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_acceleration_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_acceleration_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_angular_velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_angular_velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_angular_velocity_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_tool_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterconnectCyclic_Feedback>) istream)
  "Deserializes a message object of type '<InterconnectCyclic_Feedback>"
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
    (cl:setf (cl:slot-value msg 'imu_acceleration_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_acceleration_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_acceleration_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_angular_velocity_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_angular_velocity_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_angular_velocity_z) (roslisp-utils:decode-single-float-bits bits)))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_tool_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterconnectCyclic_Feedback>)))
  "Returns string type for a message object of type '<InterconnectCyclic_Feedback>"
  "kortex_driver/InterconnectCyclic_Feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterconnectCyclic_Feedback)))
  "Returns string type for a message object of type 'InterconnectCyclic_Feedback"
  "kortex_driver/InterconnectCyclic_Feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterconnectCyclic_Feedback>)))
  "Returns md5sum for a message object of type '<InterconnectCyclic_Feedback>"
  "927bf07b9c0ece3890499e76c04f37e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterconnectCyclic_Feedback)))
  "Returns md5sum for a message object of type 'InterconnectCyclic_Feedback"
  "927bf07b9c0ece3890499e76c04f37e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterconnectCyclic_Feedback>)))
  "Returns full string definition for message of type '<InterconnectCyclic_Feedback>"
  (cl:format cl:nil "~%InterconnectCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 jitter_comm~%float32 imu_acceleration_x~%float32 imu_acceleration_y~%float32 imu_acceleration_z~%float32 imu_angular_velocity_x~%float32 imu_angular_velocity_y~%float32 imu_angular_velocity_z~%float32 voltage~%float32 temperature_core~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%InterconnectCyclic_Feedback_tool_feedback oneof_tool_feedback~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Feedback_tool_feedback~%~%GripperCyclic_Feedback[] gripper_feedback~%================================================================================~%MSG: kortex_driver/GripperCyclic_Feedback~%~%GripperCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%MotorFeedback[] motor~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorFeedback~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterconnectCyclic_Feedback)))
  "Returns full string definition for message of type 'InterconnectCyclic_Feedback"
  (cl:format cl:nil "~%InterconnectCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 jitter_comm~%float32 imu_acceleration_x~%float32 imu_acceleration_y~%float32 imu_acceleration_z~%float32 imu_angular_velocity_x~%float32 imu_angular_velocity_y~%float32 imu_angular_velocity_z~%float32 voltage~%float32 temperature_core~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%InterconnectCyclic_Feedback_tool_feedback oneof_tool_feedback~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Feedback_tool_feedback~%~%GripperCyclic_Feedback[] gripper_feedback~%================================================================================~%MSG: kortex_driver/GripperCyclic_Feedback~%~%GripperCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%MotorFeedback[] motor~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorFeedback~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterconnectCyclic_Feedback>))
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
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_tool_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterconnectCyclic_Feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'InterconnectCyclic_Feedback
    (cl:cons ':feedback_id (feedback_id msg))
    (cl:cons ':status_flags (status_flags msg))
    (cl:cons ':jitter_comm (jitter_comm msg))
    (cl:cons ':imu_acceleration_x (imu_acceleration_x msg))
    (cl:cons ':imu_acceleration_y (imu_acceleration_y msg))
    (cl:cons ':imu_acceleration_z (imu_acceleration_z msg))
    (cl:cons ':imu_angular_velocity_x (imu_angular_velocity_x msg))
    (cl:cons ':imu_angular_velocity_y (imu_angular_velocity_y msg))
    (cl:cons ':imu_angular_velocity_z (imu_angular_velocity_z msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':temperature_core (temperature_core msg))
    (cl:cons ':fault_bank_a (fault_bank_a msg))
    (cl:cons ':fault_bank_b (fault_bank_b msg))
    (cl:cons ':warning_bank_a (warning_bank_a msg))
    (cl:cons ':warning_bank_b (warning_bank_b msg))
    (cl:cons ':oneof_tool_feedback (oneof_tool_feedback msg))
))
