; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GripperCyclic_Feedback.msg.html

(cl:defclass <GripperCyclic_Feedback> (roslisp-msg-protocol:ros-message)
  ((feedback_id
    :reader feedback_id
    :initarg :feedback_id
    :type kortex_driver-msg:GripperCyclic_MessageId
    :initform (cl:make-instance 'kortex_driver-msg:GripperCyclic_MessageId))
   (status_flags
    :reader status_flags
    :initarg :status_flags
    :type cl:integer
    :initform 0)
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
   (motor
    :reader motor
    :initarg :motor
    :type (cl:vector kortex_driver-msg:MotorFeedback)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:MotorFeedback :initial-element (cl:make-instance 'kortex_driver-msg:MotorFeedback))))
)

(cl:defclass GripperCyclic_Feedback (<GripperCyclic_Feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperCyclic_Feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperCyclic_Feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GripperCyclic_Feedback> is deprecated: use kortex_driver-msg:GripperCyclic_Feedback instead.")))

(cl:ensure-generic-function 'feedback_id-val :lambda-list '(m))
(cl:defmethod feedback_id-val ((m <GripperCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:feedback_id-val is deprecated.  Use kortex_driver-msg:feedback_id instead.")
  (feedback_id m))

(cl:ensure-generic-function 'status_flags-val :lambda-list '(m))
(cl:defmethod status_flags-val ((m <GripperCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:status_flags-val is deprecated.  Use kortex_driver-msg:status_flags instead.")
  (status_flags m))

(cl:ensure-generic-function 'fault_bank_a-val :lambda-list '(m))
(cl:defmethod fault_bank_a-val ((m <GripperCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:fault_bank_a-val is deprecated.  Use kortex_driver-msg:fault_bank_a instead.")
  (fault_bank_a m))

(cl:ensure-generic-function 'fault_bank_b-val :lambda-list '(m))
(cl:defmethod fault_bank_b-val ((m <GripperCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:fault_bank_b-val is deprecated.  Use kortex_driver-msg:fault_bank_b instead.")
  (fault_bank_b m))

(cl:ensure-generic-function 'warning_bank_a-val :lambda-list '(m))
(cl:defmethod warning_bank_a-val ((m <GripperCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_bank_a-val is deprecated.  Use kortex_driver-msg:warning_bank_a instead.")
  (warning_bank_a m))

(cl:ensure-generic-function 'warning_bank_b-val :lambda-list '(m))
(cl:defmethod warning_bank_b-val ((m <GripperCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_bank_b-val is deprecated.  Use kortex_driver-msg:warning_bank_b instead.")
  (warning_bank_b m))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <GripperCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:motor-val is deprecated.  Use kortex_driver-msg:motor instead.")
  (motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperCyclic_Feedback>) ostream)
  "Serializes a message object of type '<GripperCyclic_Feedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feedback_id) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status_flags)) ostream)
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motor))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperCyclic_Feedback>) istream)
  "Deserializes a message object of type '<GripperCyclic_Feedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'feedback_id) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status_flags)) (cl:read-byte istream))
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:MotorFeedback))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperCyclic_Feedback>)))
  "Returns string type for a message object of type '<GripperCyclic_Feedback>"
  "kortex_driver/GripperCyclic_Feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperCyclic_Feedback)))
  "Returns string type for a message object of type 'GripperCyclic_Feedback"
  "kortex_driver/GripperCyclic_Feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperCyclic_Feedback>)))
  "Returns md5sum for a message object of type '<GripperCyclic_Feedback>"
  "ec3639b6684e8f718fd3128a8215e686")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperCyclic_Feedback)))
  "Returns md5sum for a message object of type 'GripperCyclic_Feedback"
  "ec3639b6684e8f718fd3128a8215e686")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperCyclic_Feedback>)))
  "Returns full string definition for message of type '<GripperCyclic_Feedback>"
  (cl:format cl:nil "~%GripperCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%MotorFeedback[] motor~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorFeedback~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperCyclic_Feedback)))
  "Returns full string definition for message of type 'GripperCyclic_Feedback"
  (cl:format cl:nil "~%GripperCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%MotorFeedback[] motor~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorFeedback~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperCyclic_Feedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'feedback_id))
     4
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperCyclic_Feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperCyclic_Feedback
    (cl:cons ':feedback_id (feedback_id msg))
    (cl:cons ':status_flags (status_flags msg))
    (cl:cons ':fault_bank_a (fault_bank_a msg))
    (cl:cons ':fault_bank_b (fault_bank_b msg))
    (cl:cons ':warning_bank_a (warning_bank_a msg))
    (cl:cons ':warning_bank_b (warning_bank_b msg))
    (cl:cons ':motor (motor msg))
))
