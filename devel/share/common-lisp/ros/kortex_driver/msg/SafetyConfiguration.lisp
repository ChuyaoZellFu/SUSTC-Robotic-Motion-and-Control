; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyConfiguration.msg.html

(cl:defclass <SafetyConfiguration> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle))
   (error_threshold
    :reader error_threshold
    :initarg :error_threshold
    :type cl:float
    :initform 0.0)
   (warning_threshold
    :reader warning_threshold
    :initarg :warning_threshold
    :type cl:float
    :initform 0.0)
   (enable
    :reader enable
    :initarg :enable
    :type kortex_driver-msg:SafetyEnable
    :initform (cl:make-instance 'kortex_driver-msg:SafetyEnable)))
)

(cl:defclass SafetyConfiguration (<SafetyConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyConfiguration> is deprecated: use kortex_driver-msg:SafetyConfiguration instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <SafetyConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'error_threshold-val :lambda-list '(m))
(cl:defmethod error_threshold-val ((m <SafetyConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:error_threshold-val is deprecated.  Use kortex_driver-msg:error_threshold instead.")
  (error_threshold m))

(cl:ensure-generic-function 'warning_threshold-val :lambda-list '(m))
(cl:defmethod warning_threshold-val ((m <SafetyConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_threshold-val is deprecated.  Use kortex_driver-msg:warning_threshold instead.")
  (warning_threshold m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SafetyConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enable-val is deprecated.  Use kortex_driver-msg:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyConfiguration>) ostream)
  "Serializes a message object of type '<SafetyConfiguration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'warning_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'enable) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyConfiguration>) istream)
  "Deserializes a message object of type '<SafetyConfiguration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_threshold) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'warning_threshold) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'enable) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyConfiguration>)))
  "Returns string type for a message object of type '<SafetyConfiguration>"
  "kortex_driver/SafetyConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyConfiguration)))
  "Returns string type for a message object of type 'SafetyConfiguration"
  "kortex_driver/SafetyConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyConfiguration>)))
  "Returns md5sum for a message object of type '<SafetyConfiguration>"
  "42160db695be444fee95ed355f4763b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyConfiguration)))
  "Returns md5sum for a message object of type 'SafetyConfiguration"
  "42160db695be444fee95ed355f4763b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyConfiguration>)))
  "Returns full string definition for message of type '<SafetyConfiguration>"
  (cl:format cl:nil "~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyConfiguration)))
  "Returns full string definition for message of type 'SafetyConfiguration"
  (cl:format cl:nil "~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyConfiguration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'enable))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyConfiguration
    (cl:cons ':handle (handle msg))
    (cl:cons ':error_threshold (error_threshold msg))
    (cl:cons ':warning_threshold (warning_threshold msg))
    (cl:cons ':enable (enable msg))
))
