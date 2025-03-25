; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyInformation.msg.html

(cl:defclass <SafetyInformation> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle))
   (can_change_safety_state
    :reader can_change_safety_state
    :initarg :can_change_safety_state
    :type cl:boolean
    :initform cl:nil)
   (has_warning_threshold
    :reader has_warning_threshold
    :initarg :has_warning_threshold
    :type cl:boolean
    :initform cl:nil)
   (has_error_threshold
    :reader has_error_threshold
    :initarg :has_error_threshold
    :type cl:boolean
    :initform cl:nil)
   (limit_type
    :reader limit_type
    :initarg :limit_type
    :type cl:integer
    :initform 0)
   (default_warning_threshold
    :reader default_warning_threshold
    :initarg :default_warning_threshold
    :type cl:float
    :initform 0.0)
   (default_error_threshold
    :reader default_error_threshold
    :initarg :default_error_threshold
    :type cl:float
    :initform 0.0)
   (upper_hard_limit
    :reader upper_hard_limit
    :initarg :upper_hard_limit
    :type cl:float
    :initform 0.0)
   (lower_hard_limit
    :reader lower_hard_limit
    :initarg :lower_hard_limit
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0)
   (unit
    :reader unit
    :initarg :unit
    :type cl:integer
    :initform 0))
)

(cl:defclass SafetyInformation (<SafetyInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyInformation> is deprecated: use kortex_driver-msg:SafetyInformation instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'can_change_safety_state-val :lambda-list '(m))
(cl:defmethod can_change_safety_state-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:can_change_safety_state-val is deprecated.  Use kortex_driver-msg:can_change_safety_state instead.")
  (can_change_safety_state m))

(cl:ensure-generic-function 'has_warning_threshold-val :lambda-list '(m))
(cl:defmethod has_warning_threshold-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:has_warning_threshold-val is deprecated.  Use kortex_driver-msg:has_warning_threshold instead.")
  (has_warning_threshold m))

(cl:ensure-generic-function 'has_error_threshold-val :lambda-list '(m))
(cl:defmethod has_error_threshold-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:has_error_threshold-val is deprecated.  Use kortex_driver-msg:has_error_threshold instead.")
  (has_error_threshold m))

(cl:ensure-generic-function 'limit_type-val :lambda-list '(m))
(cl:defmethod limit_type-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:limit_type-val is deprecated.  Use kortex_driver-msg:limit_type instead.")
  (limit_type m))

(cl:ensure-generic-function 'default_warning_threshold-val :lambda-list '(m))
(cl:defmethod default_warning_threshold-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:default_warning_threshold-val is deprecated.  Use kortex_driver-msg:default_warning_threshold instead.")
  (default_warning_threshold m))

(cl:ensure-generic-function 'default_error_threshold-val :lambda-list '(m))
(cl:defmethod default_error_threshold-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:default_error_threshold-val is deprecated.  Use kortex_driver-msg:default_error_threshold instead.")
  (default_error_threshold m))

(cl:ensure-generic-function 'upper_hard_limit-val :lambda-list '(m))
(cl:defmethod upper_hard_limit-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:upper_hard_limit-val is deprecated.  Use kortex_driver-msg:upper_hard_limit instead.")
  (upper_hard_limit m))

(cl:ensure-generic-function 'lower_hard_limit-val :lambda-list '(m))
(cl:defmethod lower_hard_limit-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:lower_hard_limit-val is deprecated.  Use kortex_driver-msg:lower_hard_limit instead.")
  (lower_hard_limit m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:status-val is deprecated.  Use kortex_driver-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'unit-val :lambda-list '(m))
(cl:defmethod unit-val ((m <SafetyInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:unit-val is deprecated.  Use kortex_driver-msg:unit instead.")
  (unit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyInformation>) ostream)
  "Serializes a message object of type '<SafetyInformation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_change_safety_state) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_warning_threshold) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_error_threshold) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'limit_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'limit_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'limit_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'limit_type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'default_warning_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'default_error_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'upper_hard_limit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lower_hard_limit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'unit)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'unit)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'unit)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'unit)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyInformation>) istream)
  "Deserializes a message object of type '<SafetyInformation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:setf (cl:slot-value msg 'can_change_safety_state) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'has_warning_threshold) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'has_error_threshold) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'limit_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'limit_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'limit_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'limit_type)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'default_warning_threshold) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'default_error_threshold) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'upper_hard_limit) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lower_hard_limit) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'unit)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'unit)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'unit)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'unit)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyInformation>)))
  "Returns string type for a message object of type '<SafetyInformation>"
  "kortex_driver/SafetyInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyInformation)))
  "Returns string type for a message object of type 'SafetyInformation"
  "kortex_driver/SafetyInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyInformation>)))
  "Returns md5sum for a message object of type '<SafetyInformation>"
  "e8597ef9acfa23c653020b88d86d8b2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyInformation)))
  "Returns md5sum for a message object of type 'SafetyInformation"
  "e8597ef9acfa23c653020b88d86d8b2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyInformation>)))
  "Returns full string definition for message of type '<SafetyInformation>"
  (cl:format cl:nil "~%SafetyHandle handle~%bool can_change_safety_state~%bool has_warning_threshold~%bool has_error_threshold~%uint32 limit_type~%float32 default_warning_threshold~%float32 default_error_threshold~%float32 upper_hard_limit~%float32 lower_hard_limit~%uint32 status~%uint32 unit~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyInformation)))
  "Returns full string definition for message of type 'SafetyInformation"
  (cl:format cl:nil "~%SafetyHandle handle~%bool can_change_safety_state~%bool has_warning_threshold~%bool has_error_threshold~%uint32 limit_type~%float32 default_warning_threshold~%float32 default_error_threshold~%float32 upper_hard_limit~%float32 lower_hard_limit~%uint32 status~%uint32 unit~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyInformation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     1
     1
     1
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyInformation
    (cl:cons ':handle (handle msg))
    (cl:cons ':can_change_safety_state (can_change_safety_state msg))
    (cl:cons ':has_warning_threshold (has_warning_threshold msg))
    (cl:cons ':has_error_threshold (has_error_threshold msg))
    (cl:cons ':limit_type (limit_type msg))
    (cl:cons ':default_warning_threshold (default_warning_threshold msg))
    (cl:cons ':default_error_threshold (default_error_threshold msg))
    (cl:cons ':upper_hard_limit (upper_hard_limit msg))
    (cl:cons ':lower_hard_limit (lower_hard_limit msg))
    (cl:cons ':status (status msg))
    (cl:cons ':unit (unit msg))
))
